/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#define _MATH_DEFINES_DEFINED
#include <math.h>

// #include "misc/debug.h"
// #include "ISConstants.h"

#include "ISEarth.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

#define C_NEG_MG_DIV_KT0_F			-1.18558314779367E-04f		// - (M * g)  / (K * T0)
#define C_NEG_KT0_DIV_MG_F			-8.43466779922578000E+03f	// - (K * T0) / (M * g)

//_____ G L O B A L S ______________________________________________________

//_____ L O C A L   P R O T O T Y P E S ____________________________________

//_____ F U N C T I O N S __________________________________________________

#define POWA2	40680631590769.000	// = pow(6378137.0,2)
#define POWB2	40408299984661.453	// = pow(6356752.31424518,2)
#define POWA2_F	40680631590769.000f	// = pow(6378137.0,2)
#define POWB2_F	40408299984661.453f	// = pow(6356752.31424518,2)
#define ONE_MINUS_F 0.996647189335253 // (1 - f), where f = 1.0 / 298.257223563 is Earth flattening
#define E_SQ  0.006694379990141 // e2 = 1 - (1-f)*(1-f) - square of first eccentricity
#define E_SQ_f 0.006694379990141f
#define E_PRIME_SQ 0.006739496742276 // ep2 = e2 / (1 - e2)
#define E_POW4 4.481472345240464e-05 // e4 = e^4, first eccentricity power 4
#define ONE_MINUS_E_SQ 0.993305620009859 // (1 - e^2)
#define ONE_DIV_ONE_MINUS_E_SQ  1.006739496742276  // 1 / (1 - e^2)
#define ONE_DIV_E_SQ  1.493790315865963e+02 // 1/e^2
#define REQ 6378137.0         // Re - Equatorial radius, m
#define REQ_f 6378137.0f
#define REP 6356752.314245179 // Rp - Polar radius, m
#define E2xREQ 42697.67270717795 // e2 * Re
#define E2xREQdivIFE 42841.31151331153 // e2 * Re / (1 -f)
#define GEQ 9.7803253359f        // Equatorial gravity
#define K_GRAV 0.00193185265241f // defined gravity constants
#define K3_GRAV 3.0877e-6f       // 
#define K4_GRAV 4.0e-9f          //
#define K5_GRAV 7.2e-14f         //

#ifndef ECEF2LLA_METHOD
#define ECEF2LLA_METHOD 5
#endif

/* Coordinate transformation from ECEF coordinates to latitude/longitude/altitude (rad,rad,m) */
void ecef2lla(const double *Pe, double *LLA)
{
    int iter = 0;
    double p, p2;

    // Earth equatorial radius
    // Re = 6378137; // m
    // Square of first eccentricity
    // e2 = 1 - (1-f)*(1-f);

    p2 = Pe[0] * Pe[0] + Pe[1] * Pe[1];
    p  = sqrt(p2);

    // Longitude
    LLA[1] = atan2(Pe[1], Pe[0]);

    // The original Bowring's irrational geodetic-latitude (mu) equation:
    // k - 1 - e^2 * R * k / sqrt(p^2 + (1 - e^2) * z^2 * k^2) = 0
    // where k = p / z * tan(mu)
    // The latitude solution mu = atan2(k * z, p)
    // The height is then calculated as
    // h = 1 / e^2 * (1/k - 1/k0) * sqrt(p^2 + z^2 * k^2)
    // k0 = 1 / (1 - e^2)
    // Various methods can be used to solve Bowring's equation.

#if ECEF2LLA_METHOD == 0
    double Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0, w,
               F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6, z_i, z2_k_k;

    // Original Bowring's iterative procedure with additional trigonometric functions
    // which typically converges after 2 or 3 iterations
    beta = atan2(Pe[2], ONE_MINUS_F * p); // reduced latitude, initial guess
    // Precompute these values to speed-up computation
    // B = e2 * Re; // >>> this is now E2xREQ
    // A = e2 * Re / (1 - f); // >>> this is now E2xREQdivIFE
    while (fabs(err) > 1.0e-8 && iter < 5)
    {
        iter++;
        val = LLA[0];
        LLA[0] = atan2(Pe[2] + E2xREQdivIFE * pow(sin(beta), 3), p - E2xREQ * pow(cos(beta), 3));
        beta = atan(ONE_MINUS_F * tan(LLA[0]));
        err = LLA[0] - val;
    }
    // Radius of curvature in the vertical prime
    sinmu = sin(LLA[0]);
    Rn = REQ / sqrt(1.0 - E_SQ * sinmu * sinmu);
    // Altitude above planetary ellipsoid
    LLA[2] = p * cos(LLA[0]) + (Pe[2] + E_SQ * Rn * sinmu) * sinmu - Rn;

#elif ECEF2LLA_METHOD == 1
    double Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0, w,
               F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6, z_i, z2_k_k;

    // The equation can be solved by Newton - Raphson iteration method :
    // k_next = (c + (1 - e^2) * z^2 * k^3) / (c - p^2) =
    //        = 1 + (p^2 + (1 - e^2) * z^2 * k^3) / (c - p^2)
    // where
    // c = 1 / (R * e^2) * (p^2 + (1 - e^2) * z^2 * k^2) ^ (3/2)
    // Initial value k0 is a good start when h is near zero.
    // Even a single iteration produces a sufficiently accurate solution.

    double z2 = Pe[2] * Pe[2];

    k = ONE_DIV_ONE_MINUS_E_SQ;
    z2_k_k = z2 * k * k;
//    for (i = 0; i < 1; i++) {
        val = p2 + ONE_MINUS_E_SQ * z2_k_k;
        c = sqrt(val * val * val) / E2xREQ;
        k = 1 + (p2 + ONE_MINUS_E_SQ * z2_k_k * k) / (c - p2);
        z2_k_k = z2 * k * k;
//        }
    // Latitude
    LLA[0] = atan2(k * Pe[2], p);
    // Altitude above planetary ellipsoid
    LLA[2] = ONE_DIV_E_SQ * (1.0 / k - ONE_MINUS_E_SQ) * sqrt(p2 + z2_k_k);

#elif ECEF2LLA_METHOD == 2
    // The Bowring's quartic equation of k can be solved by Ferrari's
    // solution.Then compute latitude and height as above.

    double z2 = Pe[2] * Pe[2];
    zeta = ONE_MINUS_E_SQ * z2 / POWA2;
    rho = 0.166666666666667 * (p2 / POWA2 + zeta - E_POW4);
    s = E_POW4 * zeta * p2 / (4.0 * rho * rho * rho * POWA2);
    t = pow(1.0 + s + sqrt(s * (s + 2.0)), 0.333333333333333);
    u = rho * (t + 1.0 + 1.0 / t);
    v = sqrt(u * u + E_POW4 * zeta);
    w = E_SQ * (u + v - zeta) / (2.0 * v);
    k = 1.0 + E_SQ * (sqrt(u + v + w * w) + w) / (u + v);
    // Latitude
    LLA[0] = atan2(k * Pe[2], p);
    // Altitude above planetary ellipsoid :
    LLA[2] = ONE_DIV_E_SQ * (1.0 / k - ONE_MINUS_E_SQ) * sqrt(p2 + z2 * k * k);

#elif ECEF2LLA_METHOD == 3
    double Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0, w,
               F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6, z_i, z2_k_k;

    double z2 = Pe[2] * Pe[2];

    // The Heikkinen's procedure using Ferrari's solution(see case 2 above)
    F = 54.0 * POWB2 * z2;
    G = p2 + ONE_MINUS_E_SQ * z2 - E_SQ * (POWA2 - POWB2);
    G2 = G * G;
    c = E_POW4 * F * p2 / (G2 * G);
    s = pow(1.0 + c + sqrt(c * (c + 2)), 0.333333333333333);
    k = s + 1.0 + 1.0 / s;
    P = F / (3.0 * k * k * G2);
    Q = sqrt(1.0 + 2.0 * E_POW4 * P);
    val = MAX(0.0, 0.5 * POWA2 * (1.0 + 1.0 / Q) - P * z2 * ONE_MINUS_E_SQ / (Q * (1.0 + Q)) - 0.5 * P * p2);
    r0 = -(P * p * E_SQ) / (1.0 + Q) + sqrt(val);
    val = p - E_SQ * r0;
    U = sqrt(MAX(0.0, val * val + z2));
    V = sqrt(MAX(0.0, val * val + ONE_MINUS_E_SQ * z2));
    z0 = POWB2 * Pe[2] / MAX(REQ * V, EPS);
    // Latitude
    LLA[0] = atan2(Pe[2] + E_PRIME_SQ * z0, p);
    // Altitude above planetary ellipsoid :
    LLA[2] = U * (1.0 - POWB2 / MAX(REQ * V, EPS));
    // Avoid numerical issues at poles
    if (V < EPS) {
        LLA[0] = LLA[0] < 0.0 ? -M_HALFPI : M_HALFPI;
        LLA[2] = fabs(Pe[2]) - REP;
    }

#elif ECEF2LLA_METHOD == 4
    double pRn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0, w,
               F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6, z_i, z2_k_k;

    beta = atan2(REQ * Pe[2], REP * p);
    LLA[0] = atan2(Pe[2] + E_PRIME_SQ * REP * pow(sin(beta), 3), p - E_SQ * REQ * pow(cos(beta), 3));
    c = REQ / sqrt(1.0 - E_SQ * pow(sin(LLA[0]), 2));
    LLA[2] = p / cos(LLA[0]) - c;
    // Correct for numerical instability in altitude near poles.
    // After this correction, error is about 2 millimeters, which is about the same as the numerical precision of the overall function
    if (fabs(Pe[1]) < 1.0 && fabs(Pe[2]) < 1.0)
        LLA[2] = fabs(Pe[2]) - REP;

#elif ECEF2LLA_METHOD == 5
    double sinmu, v = 0.0, val, err = 1.0e6, z_i;

    z_i = Pe[2];
    while (fabs(err) > 1e-4 && iter < 10)
    {
        iter++;
        val = z_i;
        sinmu = z_i / sqrt(p2 + z_i * z_i);
        v = REQ / sqrt(1.0 - E_SQ * sinmu * sinmu);
        z_i = Pe[2] + v * E_SQ * sinmu;
        err = z_i - val;
    }
    LLA[0] = atan2(z_i, p);
    // Correct for numerical instability in altitude near poles
    if (fabs(Pe[1]) < 1.0 && fabs(Pe[2]) < 1.0) {
        LLA[0] = LLA[0] < 0.0 ? -C_PIDIV2 : C_PIDIV2;
    }
    LLA[2] = sqrt(p2 + z_i * z_i) - v;

#endif
}

void ecef2lla_f(const float *Pe, float *LLA)
{
    int iter = 0;
    float p, p2;

    // Earth equatorial radius
    // Re = 6378137; // m
    // Square of first eccentricity
    // e2 = 1 - (1-f)*(1-f);

    p2 = Pe[0] * Pe[0] + Pe[1] * Pe[1];
    p = sqrtf(p2);

    // Longitude
    LLA[1] = atan2f(Pe[1], Pe[0]);

    // The original Bowring's irrational geodetic-latitude (mu) equation:
    // k - 1 - e^2 * R * k / sqrt(p^2 + (1 - e^2) * z^2 * k^2) = 0
    // where k = p / z * tan(mu)
    // The latitude solution mu = atan2(k * z, p)
    // The height is then calculated as
    // h = 1 / e^2 * (1/k - 1/k0) * sqrt(p^2 + z^2 * k^2)
    // k0 = 1 / (1 - e^2)
    // Various methods can be used to solve Bowring's equation.

#if ECEF2LLA_METHOD == 0
    float Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0f, w,
        F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6f, z_i, z2_k_k;

    // Original Bowring's iterative procedure with additional trigonometric functions
    // which typically converges after 2 or 3 iterations
    beta = atan2f(Pe[2], ONE_MINUS_F * p); // reduced latitude, initial guess
    // Precompute these values to speed-up computation
    // B = e2 * Re; // >>> this is now E2xREQ
    // A = e2 * Re / (1 - f); // >>> this is now E2xREQdivIFE
    while (fabsf(err) > 1.0e-8f && iter < 5)
    {
        iter++;
        val = LLA[0];
        LLA[0] = atan2f(Pe[2] + E2xREQdivIFE * powf(sinf(beta), 3), p - E2xREQ * powf(cosf(beta), 3));
        beta = atanf(ONE_MINUS_F * tan(LLA[0]));
        err = LLA[0] - val;
    }
    // Radius of curvature in the vertical prime
    sinmu = sinf(LLA[0]);
    Rn = REQ / sqrtf(1.0f - E_SQ * sinmu * sinmu);
    // Altitude above planetary ellipsoid
    LLA[2] = p * cosf(LLA[0]) + (Pe[2] + E_SQ * Rn * sinmu) * sinmu - Rn;

#elif ECEF2LLA_METHOD == 1
    float Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0f, w,
        F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6f, z_i, z2_k_k;

    // The equation can be solved by Newton - Raphson iteration method :
    // k_next = (c + (1 - e^2) * z^2 * k^3) / (c - p^2) =
    //        = 1 + (p^2 + (1 - e^2) * z^2 * k^3) / (c - p^2)
    // where
    // c = 1 / (R * e^2) * (p^2 + (1 - e^2) * z^2 * k^2) ^ (3/2)
    // Initial value k0 is a good start when h is near zero.
    // Even a single iteration produces a sufficiently accurate solution.

    float z2 = Pe[2] * Pe[2];

    k = ONE_DIV_ONE_MINUS_E_SQ;
    z2_k_k = z2 * k * k;
    //    for (i = 0; i < 1; i++) {
    val = p2 + ONE_MINUS_E_SQ * z2_k_k;
    c = sqrtf(val * val * val) / E2xREQ;
    k = 1 + (p2 + ONE_MINUS_E_SQ * z2_k_k * k) / (c - p2);
    z2_k_k = z2 * k * k;
    //        }
        // Latitude
    LLA[0] = atan2f(k * Pe[2], p);
    // Altitude above planetary ellipsoid
    LLA[2] = ONE_DIV_E_SQ * (1.0f / k - ONE_MINUS_E_SQ) * sqrtf(p2 + z2_k_k);

#elif ECEF2LLA_METHOD == 2
    // The Bowring's quartic equation of k can be solved by Ferrari's
    // solution.Then compute latitude and height as above.

    float z2 = Pe[2] * Pe[2];
    zeta = ONE_MINUS_E_SQ * z2 / POWA2;
    rho = 0.166666666666667f * (p2 / POWA2 + zeta - E_POW4);
    s = E_POW4 * zeta * p2 / (4.0f * rho * rho * rho * POWA2);
    t = powf(1.0f + s + sqrtf(s * (s + 2.0f)), 0.333333333333333f);
    u = rho * (t + 1.0f + 1.0f / t);
    v = sqrtf(u * u + E_POW4 * zeta);
    w = E_SQ * (u + v - zeta) / (2.0f * v);
    k = 1.0f + E_SQ * (sqrtf(u + v + w * w) + w) / (u + v);
    // Latitude
    LLA[0] = atan2f(k * Pe[2], p);
    // Altitude above planetary ellipsoid :
    LLA[2] = ONE_DIV_E_SQ * (1.0f / k - ONE_MINUS_E_SQ) * sqrtf(p2 + z2 * k * k);

#elif ECEF2LLA_METHOD == 3
    float Rn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0f, w,
        F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6f, z_i, z2_k_k;

    float z2 = Pe[2] * Pe[2];

    // The Heikkinen's procedure using Ferrari's solution(see case 2 above)
    F = 54.0f * POWB2 * z2;
    G = p2 + ONE_MINUS_E_SQ * z2 - E_SQ * (POWA2 - POWB2);
    G2 = G * G;
    c = E_POW4 * F * p2 / (G2 * G);
    s = powf(1.0f + c + sqrtf(c * (c + 2)), 0.333333333333333f);
    k = s + 1.0f + 1.0f / s;
    P = F / (3.0f * k * k * G2);
    Q = sqrtf(1.0f + 2.0f * E_POW4 * P);
    val = MAX(0.0f, 0.5f * POWA2 * (1.0f + 1.0f / Q) - P * z2 * ONE_MINUS_E_SQ / (Q * (1.0f + Q)) - 0.5f * P * p2);
    r0 = -(P * p * E_SQ) / (1.0f + Q) + sqrtf(val);
    val = p - E_SQ * r0;
    U = sqrtf(MAX(0.0f, val * val + z2));
    V = sqrtf(MAX(0.0f, val * val + ONE_MINUS_E_SQ * z2));
    z0 = POWB2 * Pe[2] / MAX(REQ * V, EPS);
    // Latitude
    LLA[0] = atan2f(Pe[2] + E_PRIME_SQ * z0, p);
    // Altitude above planetary ellipsoid :
    LLA[2] = U * (1.0f - POWB2 / MAX(REQ * V, EPS));
    // Avoid numerical issues at poles
    if (V < EPS) {
        LLA[0] = LLA[0] < 0.0f ? -M_HALFPI : M_HALFPI;
        LLA[2] = fabsf(Pe[2]) - REP;
    }

#elif ECEF2LLA_METHOD == 4
    float pRn, sinmu, beta, k, c, zeta, rho, s, t, u, v = 0.0f, w,
        F, G, G2, P, Q, val, U, V, z0, r0, err = 1.0e6f, z_i, z2_k_k;

    beta = atan2f(REQ * Pe[2], REP * p);
    LLA[0] = atan2f(Pe[2] + E_PRIME_SQ * REP * powf(sin(beta), 3), p - E_SQ * REQ * powf(cosf(beta), 3));
    c = REQ / sqrtf(1.0f - E_SQ * powf(sinf(LLA[0]), 2));
    LLA[2] = p / cosf(LLA[0]) - c;
    // Correct for numerical instability in altitude near poles.
    // After this correction, error is about 2 millimeters, which is about the same as the numerical precision of the overall function
    if (fabsf(Pe[1]) < 1.0f && fabsf(Pe[2]) < 1.0f)
        LLA[2] = fabsf(Pe[2]) - REP;

#elif ECEF2LLA_METHOD == 5
    float sinmu, v = 0.0f, val, err = 1.0e6f, z_i;

    z_i = Pe[2];
    while (fabsf(err) > 1e-4f && iter < 10)
    {
        iter++;
        val = z_i;
        sinmu = z_i / sqrtf(p2 + z_i * z_i);
        v = REQ_f / sqrtf(1.0f - E_SQ_f * sinmu * sinmu);
        z_i = Pe[2] + v * E_SQ_f * sinmu;
        err = z_i - val;
    }
    LLA[0] = atan2f(z_i, p);
    // Correct for numerical instability in altitude near poles
    if (fabsf(Pe[1]) < 1.0f && fabsf(Pe[2]) < 1.0f) {
        LLA[0] = LLA[0] < 0.0f ? -C_PIDIV2_F : C_PIDIV2_F;
    }
    LLA[2] = sqrtf(p2 + z_i * z_i) - v;

#endif
}


/* Coordinate transformation from latitude/longitude/altitude (rad,rad,m) to ECEF coordinates */
void lla2ecef(const double *LLA, double *Pe)
{
    //double e = 0.08181919084262;  // Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
    double Rn, Smu, Cmu, Sl, Cl;

    /* Earth equatorial and polar radii 
      (from flattening, f = 1/298.257223563; */
    // R = 6378137; // m
    // Earth polar radius b = R * (1-f)
    // b = 6356752.31424518;

    Smu = sin(LLA[0]);
    Cmu = cos(LLA[0]);
    Sl  = sin(LLA[1]);
    Cl  = cos(LLA[1]);
    
    // Radius of curvature at a surface point:
    Rn = REQ / sqrt(1.0 - E_SQ * Smu * Smu);

    Pe[0] = (Rn + LLA[2]) * Cmu * Cl;
    Pe[1] = (Rn + LLA[2]) * Cmu * Sl;
    Pe[2] = (ONE_MINUS_E_SQ * Rn + LLA[2]) * Smu; // Same as (Rn*POWB2/POWA2+LLA[2])*Smu but without division
}


/*
 *  Find NED (north, east, down) from LLAref to LLA (WGS-84 standard)
 *
 *  lla[0] = latitude (rad)
 *  lla[1] = longitude (rad)
 *  lla[2] = msl altitude (m)
 */
void lla2ned(ixVector3 llaRef, ixVector3 lla, ixVector3 result)
{
    ixVector3 deltaLLA;
    deltaLLA[0] = lla[0] - llaRef[0];
    deltaLLA[1] = lla[1] - llaRef[1];
    deltaLLA[2] = lla[2] - llaRef[2];
    
	// Handle longitude wrapping 
	UNWRAP_F32(deltaLLA[1]);

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
void lla2ned_d(double llaRef[3], double lla[3], ixVector3 result)
{
    ixVector3 deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping in radians
	UNWRAP_F32(deltaLLA[1]);
    
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
void llaDeg2ned_d(double llaRef[3], double lla[3], ixVector3 result)
{
    ixVector3 deltaLLA;
    deltaLLA[0] = (f_t)(lla[0] - llaRef[0]);
    deltaLLA[1] = (f_t)(lla[1] - llaRef[1]);
    deltaLLA[2] = (f_t)(lla[2] - llaRef[2]);

	// Handle longitude wrapping 
	UNWRAP_DEG_F32(deltaLLA[1]);
    
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
void ned2lla(ixVector3 ned, ixVector3 llaRef, ixVector3 result)
{
    ixVector3 deltaLLA;
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
void ned2lla_d(ixVector3 ned, double llaRef[3], double result[3])
{
    double deltaLLA[3];
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
void ned2llaDeg_d(ixVector3 ned, double llaRef[3], double result[3])
{
	double deltaLLA[3];
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
		return C_NEG_KT0_DIV_MG_F * _LOG( pKPa * C_INV_P0_KPA_F );
}


/*
 *  Find linear distance between lat,lon,alt (rad,rad,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaRadDistance( double lla1[3], double lla2[3] )
{
	ixVector3 ned;
	
	lla2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

/*
 *  Find linear distance between lat,lon,alt (deg,deg,m) coordinates.
 *
 *  return = (m) distance in meters
 */
f_t llaDegDistance( double lla1[3], double lla2[3] )
{
	ixVector3 ned;
	
	llaDeg2ned_d( lla1, lla2, ned );

	return _SQRT( ned[0]*ned[0] + ned[1]*ned[1] + ned[2]*ned[2] );
}

void ned2DeltaLla(ixVector3 ned, ixVector3 llaRef, ixVector3 deltaLLA)
{
	deltaLLA[0] =  ned[0] * INV_EARTH_RADIUS_F;
	deltaLLA[1] =  ned[1] * INV_EARTH_RADIUS_F / _COS(llaRef[0]);
	deltaLLA[2] = -ned[2];
}

void ned2DeltaLla_d(ixVector3 ned, double llaRef[3], double deltaLLA[3])
{
	deltaLLA[0] = (double)( ned[0] * INV_EARTH_RADIUS_F);
	deltaLLA[1] = (double)( ned[1] * INV_EARTH_RADIUS_F / _COS(((f_t)llaRef[0])) );
	deltaLLA[2] = (double)(-ned[2]);
}

void ned2DeltaLlaDeg_d(ixVector3 ned, double llaRef[3], double deltaLLA[3])
{
	deltaLLA[0] = (double)(ned[0] * INV_EARTH_RADIUS_F * C_RAD2DEG_F);
	deltaLLA[1] = (double)(ned[1] * INV_EARTH_RADIUS_F * C_RAD2DEG_F / _COS( (((f_t)llaRef[0])*C_DEG2RAD_F) ) );
	deltaLLA[2] = (double)(-ned[2]);
}

// Convert LLA from radians to degrees
void lla_Rad2Deg_d(double result[3], double lla[3])
{
	result[0] = C_RAD2DEG * lla[0];
	result[1] = C_RAD2DEG * lla[1];
	result[2] = lla[2];
}

// Convert LLA from degrees to radians
void lla_Deg2Rad_d(double result[3], double lla[3])
{
	result[0] = C_DEG2RAD * lla[0];
	result[1] = C_DEG2RAD * lla[1];
	result[2] = lla[2];
}

void lla_Deg2Rad_d2(double result[3], double lat, double lon, double alt)
{
	result[0] = C_DEG2RAD * lat;
	result[1] = C_DEG2RAD * lon;
	result[2] = alt;
}

/*
 *  Check if lat,lon,alt (deg,deg,m) coordinates are valid.
 *
 *  return 1 on success, 0 on failure.
 */
int llaDegValid( double lla[3] )
{
    return 
        (fabs(lla[0]) <= 90.0) &&           // Lat
        (fabs(lla[1]) <= 180.0) &&          // Lon
        (fabs(lla[2]) <= INS_MAX_ALTITUDE); // Alt: -10 to 50 km
}


/* IGF-80 gravity model with WGS-84 ellipsoid refinement */
float gravity_igf80(float lat_rad, float alt)
{
    float g0, sinmu2;

    // Equatorial gravity
    // float ge = 9.7803253359f;
    // Defined constant k = (b*gp - a*ge) / a / ge;
    // double k = 0.00193185265241;
    // Square of first eccentricity e^2 = 1 - (1 - f)^2 = 1 - (b/a)^2;
    // double e2 = 0.00669437999013;

    sinmu2 = sinf(lat_rad);
    sinmu2 *= sinmu2;
    g0 = GEQ * (1.0f + K_GRAV * sinmu2) / sqrtf(1.0f - E_SQ_f * sinmu2);

    // Free air correction
    return g0 - (K3_GRAV - K4_GRAV * sinmu2 - K5_GRAV * alt) * alt;
}


/*
* Quaternion rotation to NED with respect to ECEF at specified LLA
*/
// void quatEcef2Ned(ixVector4 Qe2n, const ixVector3d lla)
// {
// 	ixVector3 Ee2nLL;
// 	ixVector4 Qe2n0LL, Qe2nLL;
// 
// 	//Qe2n0LL is reference attitude [NED w/r/t ECEF] at zero latitude and longitude (elsewhere qned0)
// 	Qe2n0LL[0] = cosf(-C_PIDIV4_F);
// 	Qe2n0LL[1] = 0.0f;
// 	Qe2n0LL[2] = sinf(-C_PIDIV4_F);
// 	Qe2n0LL[3] = 0.0f;
// 
// 	//Qe2nLL is delta reference attitude [NED w/r/t ECEF] accounting for latitude and longitude (elsewhere qned)
// 	Ee2nLL[0] = 0.0f;
// 	Ee2nLL[1] = (float)(-lla[0]);
// 	Ee2nLL[2] = (float)(lla[1]);
// 
// 	euler2quat(Ee2nLL, Qe2nLL);
// 
// 	//Qe2b=Qe2n*Qn2b is vehicle attitude [BOD w/r/t ECEF]
// 	mul_Quat_Quat(Qe2n, Qe2n0LL, Qe2nLL);
// }


/* Attitude quaternion for NED frame in ECEF */
void quat_ecef2ned(float lat, float lon, float *qe2n)
{
    float eul[3];

    eul[0] = 0.0f;
    eul[1] = -lat - 0.5f * C_PI_F;
    eul[2] = lon;
    euler2quat(eul, qe2n);
}


/*
* Convert ECEF quaternion to NED euler at specified ECEF
*/
void qe2b2EulerNedEcef(ixVector3 eul, const ixVector4 qe2b, const ixVector3d ecef)
{
	ixVector3d lla;

// 	ecef2lla_d(ecef, lla);
	ecef2lla(ecef, lla);
	qe2b2EulerNedLLA(eul, qe2b, lla);
}


/**
 * @brief primeRadius
 * @param lat
 * @return
 */
double primeRadius(const double lat)
{
    double slat = sin(lat);
    return REQ / sqrt(1.0-E_SQ*slat*slat);
}


/**
 * @brief meridonalRadius
 * @param lat
 * @return
 */
double meridonalRadius(const double lat)
{
    double slat = sin(lat);
    double Rprime = primeRadius(lat);
    return Rprime * (1.0-E_SQ) / (1.0-E_SQ*slat*slat);
}


/**
 * @brief rangeBearing_from_lla
 * @param lla1
 * @param lla2
 * @param rb
 *
 * Compute the range and bearing by principle of the osculating sphere: We define a sphere that is tangential to the
 * WGS84 ellipsoid at latitude 1 (the latitude given in lla1). Then we perform the range and bearing computation on
 * that sphere instead of on the ellipsoid itself (which requires more complex math).
 *
 */
void rangeBearing_from_lla(const ixVector3d lla1, const ixVector3d lla2, ixVector2d rb)
{
    double lat1 = lla1[0];
    double lon1 = lla1[1];
    double lat2 = lla2[0];
    double lon2 = lla2[1];
    // Principle of osculating sphere: computing spherical angles on ellipsoids is very hard to do, so instead we compute them on a
    // sphere that is tangential to the ellipsoid at lat1 (i.e. has the same radius of curvature in both directions). By going to a sphere
    // the concept of latitude has some special conetations: the latitude is the spherical angle that the radius of curvature makes with
    // the equatorial plane. For a perfect sphere, this concides with the center of the sphere. So we have to translate the (given)
    // ellispoidal latitudes (lat1 and lat2) to their spherical equivalent as per soculating sphere. For lat1, it turns out (because the
    // osculating sphere is tangential at lat1) that it's equivalent to the spherical latitude. But for the second latitude, we have to
    // compute the equivalent spherical latitude. This is done by making the assumption that the distance along the meridian between lat1
    // and lat2 over the ellipsoid is the same as the distance between spherical lat1 and spherical lat2 along the sphere. So we can
    // equate the definate integral between ellipsoidal lat1 and lat 2 of the meridional radius of curvature to the definate integral
    // between spherical lat1 and pherical lat 2 over the prime radius of curvature over.
    // This results in Rmeridional * (lat2 - lat1) = Rprime * (spherical_lat2 - lat1)
    // Then form this expression we can compute the spherical_lat2 value
    //
    double Rprime = primeRadius(lat1);
    double avgLat = 0.5*(lat1 + lat2);
    double Rmeridional = meridonalRadius(avgLat);
    double sphericalLat2 = lat1 + Rmeridional/Rprime*(lat2 - lat1);

    // Now instead of using lat2, we are going to use spherical lat2, and pretend the earth is a perfect sphere and then we just
    // complete the spherical triangle through some standard highschool math.
    //
    double deltaLon = lon2 - lon1;
    double cosLat1 = cos(lat1);
    double sinLat1 = sin(lat1);
    double cosLat2 = cos(sphericalLat2);
    double sinLat2 = sin(sphericalLat2);
    double cosDeltaLon = cos(deltaLon);

    double x = cosLat1 * sinLat2 - sinLat1 * cosLat2 * cosDeltaLon;
    double y = cosLat2 * sin(deltaLon);
    double z = sinLat1 * sinLat2 + cosLat1 * cosLat2 * cosDeltaLon;

    double angular_range = atan2(sqrt(x*x + y*y), z);
    rb[0] =  angular_range * Rprime;
    rb[1] = atan2(y,x);
}

/* Coordinate transformation matrix from NED to ECEF frame */

void rotMat_ned2ecef(const double *latlon, float *R)
{
	double Smu, Cmu, Sl, Cl;

    Smu = sin(latlon[0]);
    Cmu = cos(latlon[0]);
    Sl  = sin(latlon[1]);
    Cl  = cos(latlon[1]);

    R[0] = -(float)(Smu * Cl);
    R[1] = -(float)Sl;
    R[2] = -(float)(Cmu * Cl);
    R[3] = -(float)(Smu * Sl);
    R[4] =  (float)Cl;
    R[5] = -(float)(Cmu * Sl);
    R[6] =  (float)Cmu;
    R[7] =  0.0f;
    R[8] = -(float)Smu;

}

// vertVel is positive in the up direction
void gndSpeedToVelEcef(const float gndSpeed, const float hdg, const float vertVel, const ixVector3d lla, ixVector3 velEcef)
{
    ixVector3 velNed;
    ixMatrix3 Rn2e;

    velNed[0] = cosf(hdg) * gndSpeed;
    velNed[1] = sinf(hdg) * gndSpeed;
    velNed[2] = -vertVel;

    rotMat_ned2ecef(lla, Rn2e);

    mul_Mat3x3_Vec3x1(velEcef, Rn2e, velNed);
}
