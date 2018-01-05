/*
 *  ISEarth.h
 *
 *  Created on: Sept 4, 2011
 *      Author: waltj
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

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

#define DEG2RAD_EARTH_RADIUS_F		111120.0f					// = DEG2RAD * earth_radius_in_meters
#define INV_DEG2RAD_EARTH_RADIUS_F	8.99928005759539236861e-6f	// = 1 / ( DEG2RAD * earth_radius_in_meters )

#define EARTH_RADIUS_F			6366707.01949371f				// = earth_radius_in_meters
#define INV_EARTH_RADIUS_F		1.5706706731410E-07f				// = 1 / earth_radius_in_meters

#define UNWRAP_DEG_F64(x)		{while( (x) > (180.0) )	 (x) -= (360.0);     while( (x) < (-180.0) )  (x) += (360.0);}	    // unwrap to +- 180
#define UNWRAP_DEG_F(x)			{while( (x) > (180.0f) ) (x) -= (360.0f);    while( (x) < (-180.0f) ) (x) += (360.0f);}	    // unwrap to +- 180
#define UNWRAP_F64(x)			{while( (x) > (C_PI) )   (x) -= (C_TWOPI);   while( (x) < (-C_PI) )   (x) += (C_TWOPI);}	// unwrap to +- PI
#define UNWRAP_F(x)				{while( (x) > (C_PI_F) ) (x) -= (C_TWOPI_F); while( (x) < (-C_PI_F) ) (x) += (C_TWOPI_F);}	// unwrap to +- PI

#if 0
typedef Vector2     Vector2_t;
typedef Vector3     Vector3_t;
typedef Vector4     Vector4_t;
typedef Vector3_t   Euler_t;        // phi, theta, psi (roll, pitch, yaw)
typedef Vector4_t   Quat_t;         // w, x, y, z
typedef Matrix2     Matrix2_t;
typedef Matrix3     Matrix3_t;
typedef Matrix4     Matrix4_t;
#else

#endif


#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          // allow "inline" keyword to work in windows w/ c program
#endif


//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________


// Coordinate transformation from latitude/longitude/altitude (rad,rad,m) to ECEF coordinates
void lla2ecef_d(double *lla, double *ecef);

// Coordinate transformation from ECEF coordinates to latitude/longitude/altitude (rad,rad,m)
void ecef2lla_d(double *ecef, double *lla);

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned( Vector3_t llaRef, Vector3_t lla, Vector3_t result );
void lla2ned_d( Vector3d llaRef, Vector3d lla, Vector3_t result );     // double precision

/*
 *  Find NED (north, east, down) from LLAref to LLA
 *
 *  lla[0] = latitude (degrees)
 *  lla[1] = longitude (degrees)
 *  lla[2] = msl altitude (m)
 */
void llaDeg2ned_d(Vector3d llaRef, Vector3d lla, Vector3_t result);

/*
 *  Find LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla( Vector3_t ned, Vector3_t llaRef, Vector3_t result );
void ned2lla_d( Vector3_t ned, Vector3d llaRef, Vector3d result );     // double precision

/*
*  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2llaDeg_d(Vector3_t ned, Vector3d llaRef, Vector3d result);

/*
 *  Find Delta LLA of NED (north, east, down) from LLAref
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2DeltaLla(Vector3_t ned, Vector3 llaRef, Vector3 deltaLLA);
void ned2DeltaLla_d(Vector3_t ned, Vector3d llaRef, Vector3d deltaLLA);

/*
*  Find Delta LLA of NED (north, east, down) from LLAref
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2DeltaLlaDeg_d(Vector3_t ned, Vector3d llaRef, Vector3d deltaLLA);

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(Vector3d result, Vector3d lla);

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(Vector3d result, Vector3d lla);
void lla_Deg2Rad_d2(Vector3d result, double lat, double lon, double alt);

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
f_t llaRadDistance( Vector3d lla1, Vector3d lla2 );
f_t llaDegDistance( Vector3d lla1, Vector3d lla2 );


#ifdef __cplusplus
}
#endif

#endif /* ISEARTH_H_ */
