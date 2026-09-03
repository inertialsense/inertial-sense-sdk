/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISEarth.h
 * @brief Earth-relative coordinate transformations: ECEF/LLA/NED conversions, range/bearing,
 * radius-of-curvature, gravity model, and GPS time helpers (gtime_t).
 *
 * Unless otherwise noted: angles are in radians (functions with a "Deg" in their name take/return
 * degrees instead), distances/altitudes are in meters, and LLA is (latitude, longitude, altitude).
 * ECEF is earth-centered earth-fixed; NED is North-East-Down relative to a reference LLA.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef ISEARTH_H_
#define ISEARTH_H_

#include "ISMatrix.h"
#include "ISConstants.h"
#include "ISPose.h"
#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEG2RAD_EARTH_RADIUS_F      111120.0f                    //!< = DEG2RAD * earth_radius_in_meters
#define INV_DEG2RAD_EARTH_RADIUS_F  8.99928005759539236861e-6f    //!< = 1 / (DEG2RAD * earth_radius_in_meters)

#define EARTH_RADIUS_F              6366707.01949371f                //!< = earth_radius_in_meters
#define INV_EARTH_RADIUS_F          1.5706706731410E-07f                //!< = 1 / earth_radius_in_meters

#ifndef MAX
#define MAX(a,b)    (((a) > (b)) ? (a) : (b))    //!< Standard two-argument maximum, defined here if not already available
#endif

#if 0
typedef ixVector2     ixVector2;
typedef ixVector3     ixVector3;
typedef ixVector4     ixVector4;
typedef ixVector3     ixEuler;        // phi, theta, psi (roll, pitch, yaw)
typedef ixVector4     ixQuat;         // w, x, y, z
typedef ixMatrix2     ixMatrix2;
typedef ixMatrix3     ixMatrix3;
typedef ixMatrix4     ixMatrix4;
#else

#endif


#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          //!< Allow the "inline" keyword to work in Windows w/ a C (non-C++) program
#endif


//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________

/**
 * @brief Convert ECEF coordinates to latitude/longitude/altitude, double precision.
 * @param Pe  ECEF position (X, Y, Z), meters.
 * @param LLA Output: latitude (rad), longitude (rad), altitude (m).
 */
void ecef2lla(const double *Pe, double *LLA);

/**
 * @brief Convert ECEF coordinates to latitude/longitude/altitude, single precision.
 * @param Pe  ECEF position (X, Y, Z), meters.
 * @param LLA Output: latitude (rad), longitude (rad), altitude (m).
 */
void ecef2lla_f(const float *Pe, float *LLA);

/**
 * @brief Convert latitude/longitude/altitude to ECEF coordinates.
 * @param LLA Latitude (rad), longitude (rad), altitude (m).
 * @param Pe  Output: ECEF position (X, Y, Z), meters.
 */
void lla2ecef(const double *LLA, double *Pe);

/**
 * @brief Find NED (north, east, down) offset from llaRef to lla, single precision.
 * @param llaRef Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param lla    Target latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: NED offset (meters) from llaRef to lla.
 */
void lla2ned(ixVector3 llaRef, ixVector3 lla, ixVector3 result);

/**
 * @brief Find NED (north, east, down) offset from llaRef to lla, double precision.
 * @param llaRef Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param lla    Target latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: NED offset (meters) from llaRef to lla.
 */
void lla2ned_d(double llaRef[3], double lla[3], ixVector3 result);

/**
 * @brief Find NED (north, east, down) offset from llaRef to lla, double precision, degrees input.
 * @param llaRef Reference latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param lla    Target latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: NED offset (meters) from llaRef to lla.
 */
void llaDeg2ned_d(double llaRef[3], double lla[3], ixVector3 result);

/**
 * @brief Find LLA from an NED (north, east, down) offset relative to llaRef, single precision.
 * @param ned    NED offset (meters) from llaRef.
 * @param llaRef Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 */
void ned2lla(ixVector3 ned, ixVector3 llaRef, ixVector3 result);

/**
 * @brief Find LLA from an NED (north, east, down) offset relative to llaRef, double precision.
 * @param ned    NED offset (meters) from llaRef.
 * @param llaRef Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 */
void ned2lla_d(ixVector3 ned, double llaRef[3], double result[3]);

/**
 * @brief Find LLA from an NED (north, east, down) offset relative to llaRef, double precision, degrees output (WGS-84 standard).
 * @param ned    NED offset (meters) from llaRef.
 * @param llaRef Reference latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param result Output: latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 */
void ned2llaDeg_d(ixVector3 ned, double llaRef[3], double result[3]);

/**
 * @brief Find the delta-LLA equivalent of an NED (north, east, down) offset relative to llaRef, single precision.
 * @param ned     NED offset (meters) from llaRef.
 * @param llaRef  Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param deltaLLA Output: delta latitude (rad), delta longitude (rad), delta altitude (m).
 */
void ned2DeltaLla(ixVector3 ned, ixVector3 llaRef, ixVector3 deltaLLA);

/**
 * @brief Find the delta-LLA equivalent of an NED (north, east, down) offset relative to llaRef, double precision.
 * @param ned     NED offset (meters) from llaRef.
 * @param llaRef  Reference latitude (rad), longitude (rad), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param deltaLLA Output: delta latitude (rad), delta longitude (rad), delta altitude (m).
 */
void ned2DeltaLla_d(ixVector3 ned, double llaRef[3], double deltaLLA[3]);

/**
 * @brief Find the delta-LLA equivalent of an NED (north, east, down) offset relative to llaRef, double precision, degrees output.
 * @param ned     NED offset (meters) from llaRef.
 * @param llaRef  Reference latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param deltaLLA Output: delta latitude (degrees), delta longitude (degrees), delta altitude (m).
 */
void ned2DeltaLlaDeg_d(ixVector3 ned, double llaRef[3], double deltaLLA[3]);

/**
 * @brief Find the ECEF position given NED (north, east, down) offset relative to llaRef, double precision.
 * @param ned     NED offset (meters) from llaRef.
 * @param llaRefDeg Reference latitude (degrees), longitude (degrees), altitude (m; MSL or WGS-84, any datum consistent between llaRef and lla/result).
 * @param ecef Output: ECEF position (m).
 */
void ned2ecef_d(const ixVector3 ned, const double llaRefDeg[3], double ecef[3]);

/**
 * @brief Convert LLA from radians to degrees (altitude is passed through unchanged).
 * @param result Output: latitude (degrees), longitude (degrees), altitude (m).
 * @param lla    Input: latitude (rad), longitude (rad), altitude (m).
 */
void lla_Rad2Deg_d(double result[3], double lla[3]);

/**
 * @brief Convert LLA from degrees to radians (altitude is passed through unchanged).
 * @param result Output: latitude (rad), longitude (rad), altitude (m).
 * @param lla    Input: latitude (degrees), longitude (degrees), altitude (m).
 */
void lla_Deg2Rad_d(double result[3], double lla[3]);

/**
 * @brief Convert individual lat/lon/alt from degrees to radians (altitude is passed through unchanged).
 * @param result Output: latitude (rad), longitude (rad), altitude (m).
 * @param lat    Input latitude (degrees).
 * @param lon    Input longitude (degrees).
 * @param alt    Input altitude (m).
 */
void lla_Deg2Rad_d2(double result[3], double lat, double lon, double alt);

/**
 * @brief Find MSL altitude based on barometric pressure. See https://en.wikipedia.org/wiki/Atmospheric_pressure
 * @param pKPa Barometric pressure (kPa).
 * @return MSL altitude (m).
 */
f_t baro2msl(f_t pKPa);

/**
 * @brief Find the linear (great-circle-adjacent, ECEF-based) distance between two lat/lon/alt coordinates, radians input.
 * @param lla1 First coordinate: latitude (rad), longitude (rad), altitude (m).
 * @param lla2 Second coordinate: latitude (rad), longitude (rad), altitude (m).
 * @return Distance (m).
 */
f_t llaRadDistance(double lla1[3], double lla2[3]);

/**
 * @brief Find the linear (great-circle-adjacent, ECEF-based) distance between two lat/lon/alt coordinates, degrees input.
 * @param lla1 First coordinate: latitude (degrees), longitude (degrees), altitude (m).
 * @param lla2 Second coordinate: latitude (degrees), longitude (degrees), altitude (m).
 * @return Distance (m).
 */
f_t llaDegDistance(double lla1[3], double lla2[3]);

/**
 * @brief Check whether lat/lon/alt coordinates (degrees) are within valid ranges.
 * @param lla Latitude (degrees), longitude (degrees), altitude (m).
 * @return 0 on success (valid), -1 on failure (invalid).
 */
int llaDegValid(double lla[3]);

/**
 * @brief IGF-80 gravity model with WGS-84 ellipsoid refinement.
 * @param lat Latitude (rad).
 * @param alt Altitude (m).
 * @return Local gravitational acceleration (m/s^2).
 */
float gravity_igf80(float lat, float alt);

// Quaternion rotation to NED with respect to ECEF at specified LLA
// void quatEcef2Ned(ixVector4 Qe2n, const ixVector3d lla);

/**
 * @brief Compute the attitude quaternion for the NED frame, expressed in ECEF, at a given latitude/longitude.
 * @param lat  Latitude (rad).
 * @param lon  Longitude (rad).
 * @param qe2n Output: quaternion rotation from ECEF to NED (W, X, Y, Z).
 */
void quat_ecef2ned(float lat, float lon, float *qe2n);

/**
 * @brief Convert an ECEF body-rotation quaternion to NED Euler angles at a specified ECEF position.
 * @param theta Output: Euler angles (roll, pitch, yaw), radians, w.r.t. NED.
 * @param qe2b  Input: quaternion body rotation w.r.t. ECEF (W, X, Y, Z).
 * @param ecef  Input: ECEF position (X, Y, Z), meters, used to determine the local NED frame.
 */
void qe2b2EulerNedEcef(ixVector3 theta, const ixVector4 qe2b, const ixVector3d ecef);

/**
 * @brief Convert an ECEF body-rotation quaternion to NED Euler angles at a specified LLA position.
 * @param eul  Output: Euler angles (roll, pitch, yaw), radians, w.r.t. NED.
 * @param qe2b Input: quaternion body rotation w.r.t. ECEF (W, X, Y, Z).
 * @param lla  Input: latitude (rad), longitude (rad), altitude (m), used to determine the local NED frame.
 */
void qe2b2EulerNedLLA(ixVector3 eul, const ixVector4 qe2b, const ixVector3d lla);

/**
 * @brief primeRadius       Compute prime radius of curvature
 * @param lat               latitude at which we want to compute the prime radius of curvature (rad)
 * @return                  prime radius of curvature (m)
 */
double primeRadius(const double lat);

/**
 * @brief meridonalRadius   Compute meridional radius of curvature
 * @param lat               latitude at which we want to compute the merdional radius of curvature (rad)
 * @return                  meridional radius of curvature (m)
 */
double meridonalRadius(const double lat);


/**
 * @brief rangeBearing_from_lla         Compute the range and bearing between two geodetic coordinates
 * @param lla1                          array of lat (rad), lon (rad) and altitude (m)
 * @param lla2                          array of lat (rad), lon (rad) and altitude (m)
 * @param rb                            array of the resulting range (m) and bearing (rad)
 */
void rangeBearing_from_lla(const ixVector3d lla1, const ixVector3d lla2, ixVector2d rb);

/**
 * @brief Compute rotation matrix from NED to ECEF
 *
 * @param latlon                        latitute/longitude (rad) (2x1)
 * @param R                             rotation matrix (3x3)
 */
void rotMat_ned2ecef(const double *latlon, float *R);

/**
 * @brief Convert ground speed and heading to ECEF velocity
 *
 * @param gndSpeed                      m/s
 * @param hdg                           rad
 * @param vertVel                       m/s
 * @param lla                           rad (can just pass lat/lon, alt not required)
 * @param velEcef                       m/s
 */
void gndSpeedToVelEcef(const float gndSpeed, const float hdg, const float vertVel, const ixVector3d lla, ixVector3 velEcef);

#if !defined(RTK_EMBEDDED)
/**
 * @brief Convert week and tow in gps time to gtime_t struct
 *
 * @param week                  week number in gps time
 * @param sec                   time of week in gps time (s)
 * @return                      gtime_t struct
 */
gtime_t ISgpst2time(int week, double sec);

/**
 * @brief Convert gtime_t struct to week and tow in gps time
 * @param t                     gtime_t struct input
 * @param week                  number in gps time (NULL: no output)
 * @return                      time of week in gps time (s)
 */
double IStime2gpst(gtime_t t, int *week);

/**
 * @brief Add time to gtime_t struct
 * @param t                     gtime_t struct
 * @param sec                   time to add (s)
 * @return                      gtime_t struct (t+sec)
 */
gtime_t IStimeadd(gtime_t t, double sec);

/**
 * @brief Difference between gtime_t structs
 * @param t1                    gtime_t struct (minuend)
 * @param t2                    gtime_t struct (subtrahend)
 * @return                      time difference (t1-t2) (s)
 */
double IStimediff(gtime_t t1, gtime_t t2);

/** 
 * @brief Convert calendar day/time to gtime_t struct
 * @param ep                    Pointer to day/time {year,month,day,hour,min,sec}
 * @return                      gtime_t struct
 */
gtime_t ISepoch2time(const double *ep);

/**
 * @brief convert utc to gpstime considering leap seconds
 * @param t                     time expressed in utc
 * @return                      time expressed in gpstime
 */
gtime_t ISutc2gpst(gtime_t t);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ISEARTH_H_ */
