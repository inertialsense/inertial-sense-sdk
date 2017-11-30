/*
 *  ISEarth.c
 *
 *  Created on: May 5, 2011
 *      Author: waltj
 */

#define _MATH_DEFINES_DEFINED
#include <math.h>

// #include "misc/debug.h"
// #include "../SDK/ISmath/ISConstants.h"

#include "ISEarth.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

#define C_NEG_MG_DIV_KT0_F			-1.18558314779367E-04f		// - (M * g)  / (K * T0)
#define C_NEG_KT0_DIV_MG_F			-8.43466779922578000E+03f	// - (K * T0) / (M * g)

//_____ G L O B A L S ______________________________________________________

//_____ L O C A L   P R O T O T Y P E S ____________________________________

//_____ F U N C T I O N S __________________________________________________
static const double Ra = 6378137.0;			// (m) Earth equatorial radius
static const double Rb = 6356752.31424518;	// (m) Earth polar radius Rb = Ra * (1-f)   (from flattening, f = 1/298.257223563)

#define POWA2	40680631590769.000	// = pow(6378137.0,2)
#define POWB2	40408299984661.453	// = pow(6356752.31424518,2)

#define POWA2_F	40680631590769.000f	// = pow(6378137.0,2)
#define POWB2_F	40408299984661.453f	// = pow(6356752.31424518,2)


// Coordinate transformation from latitude/longitude/altitude (rad,rad,m) to ECEF coordinates
void lla2ecef_d(double *lla, double *ecef)
{
	double e = 0.08181919084262;  // Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
	double Rn;
	double lat, lon, alt, coslat, sinlat, coslon, sinlon;

	lat = lla[0];
	lon = lla[1];
	alt = lla[2];

	sinlat = sin(lat);
	coslat = cos(lat);
	sinlon = sin(lon);
	coslon = cos(lon);

	// Radius of curvature at a surface point:
	Rn = Ra / sqrt(1.0 - e*e*sinlat*sinlat);

	ecef[0] = (Rn + lla[2]) * coslat * coslon;
	ecef[1] = (Rn + lla[2]) * coslat * sinlon;
	ecef[2] = (Rn*Rb*Rb/(Ra*Ra) + lla[2]) * sinlat;
}


// Coordinate transformation from ECEF coordinates to latitude/longitude/altitude (rad,rad,m)
void ecef2lla_d(double *ecef, double *lla)
{
	double E, F, N, P, T, lat, lon, sinlat, coslat;

	E = (POWA2 - POWB2) / POWA2;
	F = (POWA2 - POWB2) / POWB2;
	P = sqrt(pow(ecef[0], 2) + pow(ecef[1], 2));

	T = atan((ecef[2] * Ra) / (P * Rb));

	lat = atan(
		(ecef[2] + F * Rb * pow(sin(T), 3)) /
		(P - E * Ra * pow(cos(T), 3))
	);

	lon = atan2(ecef[1], ecef[0]);

	sinlat = sin(lat);
	coslat = cos(lat);

	N = POWA2 / sqrt(
		POWA2 * pow(coslat, 2) +
		POWB2 * pow(sinlat, 2)
	);

	lla[2] = P / coslat - N;
	lla[1] = lon;
	lla[0] = lat;
}


/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned(Vector3_t llaRef, Vector3_t lla, Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = lla[0] - llaRef[0];
    deltaLLA[1] = lla[1] - llaRef[1];
    deltaLLA[2] = lla[2] - llaRef[2];
    
	// Handle longitude wrapping 
	UNWRAP_F( deltaLLA[1] );

    // Find NED
	result[0] =  deltaLLA[0] * EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * EARTH_RADIUS_F * _COS( llaRef[0] );
	result[2] = -deltaLLA[2];
}


/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned_d(Vector3d llaRef, Vector3d lla, Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping 
	UNWRAP_F( deltaLLA[1] );
    
    // Find NED
	result[0] =  deltaLLA[0] * EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * EARTH_RADIUS_F * _COS( ((f_t)llaRef[0]) );
	result[2] = -deltaLLA[2];
}

/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (deg)
 *  lla[1] = longitude (deg)
 *  lla[2] = msl altitude (m)
 */
void llaDeg2ned_d(Vector3d llaRef, Vector3d lla, Vector3_t result)
{
    Vector3_t deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping 
	UNWRAP_DEG_F( deltaLLA[1] );
    
    // Find NED
	result[0] =  deltaLLA[0] * DEG2RAD_EARTH_RADIUS_F;
	result[1] =  deltaLLA[1] * DEG2RAD_EARTH_RADIUS_F * _COS( ((f_t)llaRef[0]) * C_DEG2RAD_F );
	result[2] = -deltaLLA[2];
}


/*
 *  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla(Vector3_t ned, Vector3_t llaRef, Vector3_t result)
{
    Vector3 deltaLLA;
    ned2DeltaLla( ned, llaRef, deltaLLA );
    
    // Find LLA
    result[0] = llaRef[0] + deltaLLA[0];
    result[1] = llaRef[1] + deltaLLA[1];
    result[2] = llaRef[2] + deltaLLA[2];
}


/*
 *  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void ned2lla_d(Vector3_t ned, Vector3d llaRef, Vector3d result)
{
    Vector3d deltaLLA;
    ned2DeltaLla_d( ned, llaRef, deltaLLA );
    
    // Find LLA
	result[0] = llaRef[0] + deltaLLA[0];
	result[1] = llaRef[1] + deltaLLA[1];
	result[2] = llaRef[2] + deltaLLA[2];
}


/*
*  Find LLA of NED (north, east, down) from LLAref (WGS-84 standard)
*
*  lla[0] = latitude (degrees)
*  lla[1] = longitude (degrees)
*  lla[2] = msl altitude (m)
*/
void ned2llaDeg_d(Vector3_t ned, Vector3d llaRef, Vector3d result)
{
	Vector3d deltaLLA;
	ned2DeltaLlaDeg_d(ned, llaRef, deltaLLA);

	// Find LLA
	result[0] = llaRef[0] + deltaLLA[0];
	result[1] = llaRef[1] + deltaLLA[1];
	result[2] = llaRef[2] + deltaLLA[2];
}


/*
 *  Find msl altitude based on barometric pressure
 *  https://en.wikipedia.org/wiki/Atmospheric_pressure
 *
 *  baroKPa = (kPa) barometric pressure in kilopascals
 *  return = (m) msl altitude in meters
 */
f_t baro2msl( f_t pKPa )
{
	if( pKPa <= _ZERO )
		return _ZERO;
	else
		return (f_t)C_NEG_KT0_DIV_MG_F * _LOG( pKPa * (f_t)C_INV_P0_KPA_F );
}


/*
 *  Find linear distance between lat,lon,alt (rad,rad,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaRadDistance( Vector3d lla1, Vector3d lla2 )
{
	Vector3_t ned;
	
	lla2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

/*
 *  Find linear distance between lat,lon,alt (deg,deg,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaDegDistance( Vector3d lla1, Vector3d lla2 )
{
	Vector3_t ned;
	
	llaDeg2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

void ned2DeltaLla(Vector3_t ned, Vector3 llaRef, Vector3 deltaLLA)
{
	deltaLLA[0] =  ned[0] * INV_EARTH_RADIUS_F;
	deltaLLA[1] =  ned[1] * INV_EARTH_RADIUS_F / _COS(((f_t)llaRef[0]));
	deltaLLA[2] = -ned[2];
}

void ned2DeltaLla_d(Vector3_t ned, Vector3d llaRef, Vector3d deltaLLA)
{
	deltaLLA[0] = (double)( ned[0] * INV_EARTH_RADIUS_F);
	deltaLLA[1] = (double)( ned[1] * INV_EARTH_RADIUS_F / _COS(((f_t)llaRef[0])) );
	deltaLLA[2] = (double)(-ned[2]);
}

void ned2DeltaLlaDeg_d(Vector3_t ned, Vector3d llaRef, Vector3d deltaLLA)
{
	deltaLLA[0] = (double)(ned[0] * INV_EARTH_RADIUS_F * C_RAD2DEG_F);
	deltaLLA[1] = (double)(ned[1] * INV_EARTH_RADIUS_F * C_RAD2DEG_F / _COS( (((f_t)llaRef[0])*C_DEG2RAD_F) ) );
	deltaLLA[2] = (double)(-ned[2]);
}

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(Vector3d result, Vector3d lla)
{
	result[0] = C_RAD2DEG * lla[0];
	result[1] = C_RAD2DEG * lla[1];
	result[2] = lla[2];
}

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(Vector3d result, Vector3d lla)
{
	result[0] = C_DEG2RAD * lla[0];
	result[1] = C_DEG2RAD * lla[1];
	result[2] = lla[2];
}


void lla_Deg2Rad_d2(Vector3d result, double lat, double lon, double alt)
{
	result[0] = C_DEG2RAD * lat;
	result[1] = C_DEG2RAD * lon;
	result[2] = alt;
}

