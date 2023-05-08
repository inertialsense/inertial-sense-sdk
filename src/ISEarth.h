/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ISEARTH_H_
#define ISEARTH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "ISMatrix.h"
#include "ISConstants.h"
#include "ISPose.h"

#define DEG2RAD_EARTH_RADIUS_F		111120.0f					// = DEG2RAD * earth_radius_in_meters
#define INV_DEG2RAD_EARTH_RADIUS_F	8.99928005759539236861e-6f	// = 1 / ( DEG2RAD * earth_radius_in_meters )

#define EARTH_RADIUS_F			6366707.01949371f				// = earth_radius_in_meters
#define INV_EARTH_RADIUS_F		1.5706706731410E-07f				// = 1 / earth_radius_in_meters

#ifndef MAX
#define MAX(a,b)    (((a) > (b)) ? (a) : (b))
#endif

#if 0
typedef ixVector2     ixVector2;
typedef ixVector3     ixVector3;
typedef ixVector4     ixVector4;
typedef ixVector3   ixEuler;        // phi, theta, psi (roll, pitch, yaw)
typedef ixVector4   ixQuat;         // w, x, y, z
typedef ixMatrix2     ixMatrix2;
typedef ixMatrix3     ixMatrix3;
typedef ixMatrix4     ixMatrix4;
#else

#endif


#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          // allow "inline" keyword to work in windows w/ c program
#endif


//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________

/* 
 * Coordinate transformation from ECEF coordinates to latitude/longitude/altitude (rad,rad,m)
 */
void ecef2lla(const double *Pe, double *LLA);
void ecef2lla_f(const float *Pe, float *LLA);

/*
 * Coordinate transformation from latitude/longitude/altitude (rad,rad,m) to ECEF coordinates
 */
void lla2ecef(const double *LLA, double *Pe);

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned( ixVector3 llaRef, ixVector3 lla, ixVector3 result );
void lla2ned_d( double llaRef[3], double lla[3], ixVector3 result );     // double precision

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (degrees)
 *  lla[1] = longitude (degrees)
 *  lla[2] = msl altitude (m)
 */
void llaDeg2ned_d(double llaRef[3], double lla[3], ixVector3 result);

/*
 *  Find LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla( ixVector3 ned, ixVector3 llaRef, ixVector3 result );
void ned2lla_d( ixVector3 ned, double llaRef[3], double result[3] );     // double precision

/*
*  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2llaDeg_d(ixVector3 ned, double llaRef[3], double result[3]);

/*
 *  Find Delta LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2DeltaLla(ixVector3 ned, ixVector3 llaRef, ixVector3 deltaLLA);
void ned2DeltaLla_d(ixVector3 ned, double llaRef[3], double deltaLLA[3]);

/*
*  Find Delta LLA of NED (north, east, down) from LLAref
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2DeltaLlaDeg_d(ixVector3 ned, double llaRef[3], double deltaLLA[3]);

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(double result[3], double lla[3]);

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(double result[3], double lla[3]);
void lla_Deg2Rad_d2(double result[3], double lat, double lon, double alt);

/*
 *  Find msl altitude based on barometric pressure
 *  https://en.wikipedia.org/wiki/Atmospheric_pressure
 *
 *  baroKPa = (kPa) barometric pressure in kilopascals
 *  return = (m) MSL altitude in meters
 */
f_t baro2msl( f_t pKPa );

/*
 *  Find linear distance between lat,lon,alt (rad,rad,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaRadDistance( double lla1[3], double lla2[3] );
f_t llaDegDistance( double lla1[3], double lla2[3] );

/*
 *  Check if lat,lon,alt (deg,deg,m) coordinates are valid.
 *
 *  return 0 on success, -1 on failure.
 */
int llaDegValid( double lla[3] );

/* 
 * IGF-80 gravity model with WGS-84 ellipsoid refinement 
*/
float gravity_igf80(float lat, float alt);

/*
 * Quaternion rotation to NED with respect to ECEF at specified LLA
 */
// void quatEcef2Ned(ixVector4 Qe2n, const ixVector3d lla);

/* Attitude quaternion for NED frame in ECEF */
void quat_ecef2ned(float lat, float lon, float *qe2n);

/*
* Convert ECEF quaternion to NED euler at specified ECEF
*/
void qe2b2EulerNedEcef(ixVector3 theta, const ixVector4 qe2b, const ixVector3d ecef);
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

#ifdef __cplusplus
}
#endif

#endif /* ISEARTH_H_ */
