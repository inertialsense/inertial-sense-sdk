/*
 * statistics.h
 *
 *  Created on: Jul 30, 2013
 *      Author: Walt Johnson
 */

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "ISMatrix.h"


//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

typedef struct
{
	ixVector3                 ave;		// mean
	ixVector3                 var;		// variance
	ixVector3                 std;		// standard deviation
	float                   aveAlph;	// alpha gain
	float                   aveBeta;	// beta  gain
	float                   varAlph;	// alpha gain
	float                   varBeta;	// beta  gain
} sRTSDVec3;

//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________


/*
 * Find the average value of the array.
 */
f_t mean( f_t *input, int size, int byteIncrement );
f_t mean_int32(int32_t *input, int size, int byteIncrement);
double mean_int64(int64_t *input, int size, int byteIncrement);
double mean(double *input, int size, int byteIncrement);

/*
 * Find the variance of the array.
 */
f_t variance(f_t *input, int size, int byteIncrement);
f_t variance_int32(int32_t *input, int size, int byteIncrement);
double variance_int64(int64_t *input, int size, int byteIncrement);
double variance(double *input, int size, int byteIncrement);
f_t variance_mean( f_t *input, f_t *ave, int size, int byteIncrement );

/*
* Sum of the squares of input minus ave.
*/
f_t delta_mean(f_t *input, int size, int byteIncrement, float ave);

/*
 * \brief Find the standard deviation of array.
 * 
 * \param input				Float pointer to start of data.
 * \param length			Number of values used in output.
 * \param byteIncrement		Number of bytes size of data structure used to increment pointer by (f_t = 4).  Used to iterate across an structure arrays. 
 */
f_t standard_deviation( f_t *input, int size, int byteIncrement );
double standard_deviation_d(double *input, int size, int byteIncrement);
f_t standard_deviation_int32(int32_t *input, int size, int byteIncrement);
double standard_deviation_int64(int64_t *input, int size, int byteIncrement);
f_t standard_deviation_mean( f_t *input, f_t *mean, int size, int byteIncrement );
void standard_deviation_Vec3( ixVector3 result, ixVector3 input, int size, int byteIncrement );
void stardard_deviation_mean_Vec3( ixVector3 result, ixVector3 input, ixVector3 mean, int size, int byteIncrement );
void mean_Vec3( ixVector3 ave, ixVector3 input, int size, int byteIncrement );

// Set ave to zero to get true RMS
f_t root_mean_squared(f_t *input, int size, int byteIncrement, float ave);


void init_realtime_std_dev_Vec3( sRTSDVec3 *s, float dt, float aveCornerFreqHz, float varCornerFreqHz, ixVector3 initVal );

void realtime_std_dev_Vec3( f_t *input, sRTSDVec3 *v );



#endif /* STATISTICS_H_ */
