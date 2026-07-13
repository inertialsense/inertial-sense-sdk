/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef CAN_COMM_H_
#define CAN_COMM_H_
#ifdef __cplusplus
extern "C" {
#endif


PUSH_PACK_1

/*GMT information*/
typedef struct PACKED
{
    /** Weeks since January 6th, 1980 */
    uint32_t        week;                                //4 bytes
    
    /** Time of week (since Sunday morning) in milliseconds, GMT */
    float           timeOfWeekMs;                        //4 bytes
    
} is_can_time;

/*Status Information*/
typedef struct PACKED
{
    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t        insStatus;                            //4 bytes

    /** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
    uint32_t        hdwStatus;                            //4 bytes
    
} is_can_ins_status;

/*Euler Angles*/
typedef struct PACKED
{
    /** Euler angles: roll, pitch, yaw in radians with respect to NED (scaled by 10000)*/
    int16_t                 theta1;                                //2 bytes (4 decimal places precision)
    int16_t                 theta2;                                //2 bytes
    int16_t                 theta3;                                //2 bytes
    //int16_t                    reserved;
} is_can_ins_euler;

/*Quaternion*/
typedef struct PACKED
{
    /** Quaternion body rotation with respect to NED: W, X, Y, Z (scaled by 10000)*/
    int16_t                 qn2b1;                                //2 bytes (4 decimal places precision)
    int16_t                 qn2b2;                                //2 bytes
    int16_t                 qn2b3;                                //2 bytes
    int16_t                 qn2b4;                                //2 bytes
} is_can_ins_quatn2b;

/*Quaternion*/
typedef struct PACKED
{
    /** Quaternion body rotation with respect to ECEF: W, X, Y, Z (scaled by 10000)*/
    int16_t                 qe2b1;                                //2 bytes (4 decimal places precision)
    int16_t                 qe2b2;                                //2 bytes
    int16_t                 qe2b3;                                //2 bytes
    int16_t                 qe2b4;                                //2 bytes
} is_can_ins_quate2b;

/*velocity in body frame*/
typedef struct PACKED
{
    /** Velocity U, V, W in meters per second (scaled by 100).  Convert to NED velocity using "vectorBodyToReference(uvw, theta, vel_ned)". */
    int16_t                 uvw1;                                //2 bytes (2 decimal places precision, max absolute value 327.67)
    int16_t                 uvw2;                                //2 bytes
    int16_t                 uvw3;                                //2 bytes
} is_can_uvw;

/** Velocity in ECEF */
typedef struct PACKED
{
    /** Velocity in ECEF (earth-centered earth-fixed) (scaled by 100) frame in meters per second */
    int16_t                 ve1;                                //2 bytes (2 decimal places precision, max absolute value 327.67)
    int16_t                 ve2;                                //2 bytes
    int16_t                 ve3;                                //2 bytes
    
} is_can_ve;

/** WGS84 latitude (degrees) */
typedef struct PACKED
{
    /** WGS84 latitude (degrees) */
    double                  lat;                                //8 bytes (more than 8 decimal places precision)
    
} is_can_ins_lat;

/** WGS84 longitude (degrees) */
typedef struct PACKED
{
    /** WGS84 longitude (degrees) */
    double                  lon;                                //8 bytes (more than 8 decimal places precision)
} is_can_ins_lon;

/** WGS84 height above ellipsoid (meters) */
typedef struct PACKED
{
    /** WGS84 height above ellipsoid (meters) */
    float                   alt;                                //4 bytes (more than 8 decimal places precision)    
    /** (see eGnssStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags */
    uint32_t                status;
} is_can_ins_alt;

/** North offset from reference LLA */
typedef struct PACKED
{
    /** North offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
    float                   ned1;                                //4 bytes
    /** East offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
    float                   ned2;                                //4 bytes
    
} is_can_north_east;

/** Down offset from reference LLA */
typedef struct PACKED
{
    /** Down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
    float                   ned3;                                //4 bytes
    /** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
    uint32_t                insStatus;                            //4 bytes
    
} is_can_down;

/** X Position in ECEF (earth-centered earth-fixed) frame in meters */
typedef struct PACKED
{
    /** X Position in ECEF (earth-centered earth-fixed) frame in meters */
    double                  ecef1;                                //8 bytes
    
} is_can_ecef_x;

/** Y Position in ECEF (earth-centered earth-fixed) frame in meters */
typedef struct PACKED
{
    /** Y Position in ECEF (earth-centered earth-fixed) frame in meters */
    double                  ecef2;                                //8 bytes
    
} is_can_ecef_y;

/** Z Position in ECEF (earth-centered earth-fixed) frame in meters */
typedef struct PACKED
{
    /** Z Position in ECEF (earth-centered earth-fixed) frame in meters */
    double                  ecef3;                                //8 bytes
    
} is_can_ecef_z;

/** Height above Mean Sea Level */
typedef struct PACKED
{
    /** Height above mean sea level (MSL) in meters */
    float                   msl;                                //4 bytes
} is_can_msl;

/**Preintegrated IMU values delta theta and delta velocity (X axis), and Integral period in body/IMU frame of accelerometer 0.**/
typedef struct PACKED
{
    int16_t                 theta0;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel0;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** Integral period in seconds for delta theta and delta velocity (scaled by 1000).*/
    uint16_t                dt;                                        //2 bytes
} is_can_preint_imu_px;

/**Preintegrated IMU values delta theta and delta velocity (Y axis), and Integral period in body/IMU frame of accelerometer 0.**/
typedef struct PACKED
{
    int16_t                 theta1;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel1;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** Integral period in seconds for delta theta and delta velocity (scaled by 1000).*/
    uint16_t                dt;                                        //2 bytes
    
} is_can_preint_imu_qy;

/**Preintegrated IMU values delta theta and delta velocity (Z axis), and Integral period in body/IMU frame of accelerometer 0.**/
typedef struct PACKED
{
    int16_t                 theta2;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel2;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** Integral period in seconds for delta theta and delta velocity (scaled by 1000).*/
    uint16_t                dt;                                        //2 bytes
} is_can_preint_imu_rz;

typedef struct PACKED
{
    int16_t                 theta0;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel0;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** IMU Status (eImuStatus) */
    uint32_t                status;                                    //4 bytes
} is_can_dual_imu_px;

typedef struct PACKED
{
    int16_t                 theta1;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel1;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** IMU Status (eImuStatus) */
    uint32_t                status;                                    //4 bytes
    
} is_can_dual_imu_qy;

typedef struct PACKED
{
    int16_t                 theta2;                                    //2 bytes (scaled by 1000 3 decimal places precision)
    int16_t                 vel2;                                    //2 bytes (scaled by 100 2 decimal places precision)
    /** IMU Status (eImuStatus) */
    uint32_t                status;                                    //4 bytes
} is_can_dual_imu_rz;

typedef struct PACKED
{
    /** (see eGnssStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags */
    uint32_t                status;                                    //4 bytes
    /** Average of all satellite carrier to noise ratios (signal strengths) that non-zero in dBHz */
    uint32_t                cnoMean;                                //4 byte
} is_can_gnss_pos_status;

typedef struct PACKED
{
    /** Ambiguity resolution ratio factor for validation */
    uint8_t                 arRatio;                                //1 byte
    /** Age of differential (seconds) */
    uint8_t                 differentialAge;                        //1 byte
    /** Distance to Base (m) */
    float                   distanceToBase;                            //4 bytes
    /** Angle from north to vectorToBase in local tangent plane. (rad) */
    int16_t                 headingToBase;                            //2 bytes (scaled by 1000 3 decimal places precision)
} is_can_gnss_rtk_rel;

typedef struct PACKED
{
        /** Euler roll, roll rates from each IMU(DID_IMU)**/
        int16_t             insRoll;                                //2 bytes (scaled by 10000 4 decimal places precision)
        int16_t             pImu1;                                    //2 bytes (scaled by 1000 3 decimal places precision)
        int16_t             pImu2;                                    //2 bytes (scaled by 1000 3 decimal places precision)
        
} is_can_roll_rollRate;

typedef union PACKED
{
    is_can_time time;
    is_can_ins_status insstatus;
    is_can_ins_euler euler;
    is_can_ins_quatn2b quatn2b;
    is_can_ins_quate2b quate2b;
    is_can_uvw uvw;
    is_can_ve ve;
    is_can_ins_lat lat;
    is_can_ins_lon lon;
    is_can_ins_alt alt;
    is_can_north_east ne;
    is_can_down down;
    is_can_ecef_x ecefx;
    is_can_ecef_y ecefy;
    is_can_ecef_z ecefz;
    is_can_msl msl;
    is_can_preint_imu_px pimupx;
    is_can_preint_imu_qy pimuqy;
    is_can_preint_imu_rz pimurz;
    is_can_dual_imu_px dimupx;
    is_can_dual_imu_qy dimuqy;
    is_can_dual_imu_rz dimurz;
    is_can_gnss_pos_status gnsspos;
    is_can_gnss_rtk_rel rtkrel;
    is_can_roll_rollRate rollrollrate;    
} is_can_payload;


POP_PACK


// ============================================================================
// CAN FD payloads — native float/double precision, one frame per DID
// Valid CAN FD payload sizes: 0-8, 12, 16, 20, 24, 32, 48, 64 bytes.
// Structs that don't land on a valid boundary are padded by the driver.
// ============================================================================
PUSH_PACK_1

/** FDCID_INS_1: key INS-1 fields in 64 bytes (exact FD DLC) */
typedef struct PACKED
{
    uint32_t    week;           // 4
    double      timeOfWeek;     // 8
    uint32_t    insStatus;      // 4
    uint32_t    hdwStatus;      // 4
    float       theta[3];       // 12
    float       uvw[3];         // 12
    double      lat;            // 8
    double      lon;            // 8
    float       alt;            // 4
} is_canfd_ins1;                // 64 bytes

/** FDCID_INS_2: native-precision NED quaternion */
typedef struct PACKED
{
    float       qn2b[4];        // 16
} is_canfd_ins2;                // 16 bytes

/** FDCID_INS_3: MSL altitude */
typedef struct PACKED
{
    float       msl;            // 4
} is_canfd_ins3;                // 4 bytes

/** FDCID_INS_4: ECEF quaternion, ECEF velocity, ECEF position (52 bytes → 64-byte FD frame) */
typedef struct PACKED
{
    float       qe2b[4];        // 16
    float       ve[3];          // 12
    double      ecef[3];        // 24
} is_canfd_ins4;                // 52 bytes

/** FDCID_PIMU: preintegrated IMU in native float (32 bytes exact FD DLC) */
typedef struct PACKED
{
    float       theta[3];       // 12
    float       vel[3];         // 12
    float       dt;             // 4
    uint32_t    status;         // 4
} is_canfd_pimu;                // 32 bytes

/** FDCID_IMU: IMU angular rate, acceleration, status (28 bytes → 32-byte FD frame) */
typedef struct PACKED
{
    float       pqr[3];         // 12
    float       acc[3];         // 12
    uint32_t    status;         // 4
} is_canfd_imu;                 // 28 bytes

/** FDCID_GNSS1_POS / FDCID_GNSS2_POS: GNSS fix status and mean CNO */
typedef struct PACKED
{
    uint32_t    status;         // 4
    float       cnoMean;        // 4 — native float (gnss_pos_t.cnoMean is float)
} is_canfd_gnss_pos;            // 8 bytes

/** FDCID_GNSS1_RTK_POS_REL / FDCID_GNSS2_RTK_CMP_REL: RTK data in native float (16 bytes) */
typedef struct PACKED
{
    float       arRatio;            // 4
    float       differentialAge;    // 4
    float       distanceToBase;     // 4
    float       headingToBase;      // 4
} is_canfd_gnss_rtk_rel;        // 16 bytes

typedef union PACKED
{
    is_canfd_ins1           ins1;
    is_canfd_ins2           ins2;
    is_canfd_ins3           ins3;
    is_canfd_ins4           ins4;
    is_canfd_pimu           pimu;
    is_canfd_imu            imu;
    is_canfd_gnss_pos       gnsspos;
    is_canfd_gnss_rtk_rel   rtkrel;
} is_canfd_payload;

POP_PACK

#ifdef __cplusplus
}
#endif

#endif // CAN_COMM_H_
