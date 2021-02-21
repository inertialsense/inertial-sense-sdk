/*
 * statistics.c
 *
 *  Created on: Jul 30, 2013
 *      Author: Walt Johnson
 */

#define _MATH_DEFINES_DEFINED
#include <math.h>
#include "filters.h"
#include "statistics.h"


//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________



f_t mean( f_t *input, int size, int byteIncrement )
{
    int i;
    f_t sum = 0;
	char *ptr = (char*)input;

    // Validate size
    if( size <= 0 )
        return 0;

    // Find Average
    for(i=0; i<size; i++)
	{
//         sum += input[i];
		sum += *((f_t*)ptr);
		ptr += byteIncrement;
	}

    return sum/size;
}

double mean_d(double *input, int size, int byteIncrement)
{
    int i;
    double sum = 0;
    char *ptr = (char*)input;

    // Validate size
    if (size <= 0)
        return 0;

    // Find Average
    for (i = 0; i < size; i++)
    {
        //         sum += input[i];
        sum += *((double*)ptr);
        ptr += byteIncrement;
    }

    return sum / size;
}


f_t mean_int32(int32_t *input, int size, int byteIncrement)
{
	int i;
	f_t sum = 0;
	char *ptr = (char*)input;

	// Validate size
	if (size <= 0)
		return 0;

	// Find Average
	for (i = 0; i < size; i++)
	{
		//         sum += input[i];
		sum += *((int32_t*)ptr);
		ptr += byteIncrement;
	}

	return sum / size;
}


double mean_int64(int64_t *input, int size, int byteIncrement)
{
	int i;
	double sum = 0;
	char *ptr = (char*)input;

	// Validate size
	if (size <= 0)
		return 0;

	// Find Average
	for (i = 0; i < size; i++)
	{
		//         sum += input[i];
		sum += *((int64_t*)ptr);
		ptr += byteIncrement;
	}

	return sum / size;
}


f_t variance( f_t *input, int size, int byteIncrement )
{
    int i;
	f_t dev;
	f_t sum = 0;
	f_t ave;
	char *ptr = (char*)input;
    
    // Validate size
    if( size <= 0 )
        return 0;

    ave = mean(input, size, byteIncrement);

    for(i=0; i<size; i++)
    {
		dev = (*((f_t*)ptr)) - ave;
		sum += dev * dev;
		ptr += byteIncrement;
	}
    
    return sum /= size;
}


double variance_d(double *input, int size, int byteIncrement)
{
    int i;
    double dev;
    double sum = 0;
    double ave;
    char *ptr = (char*)input;

    // Validate size
    if (size <= 0)
        return 0;

    ave = mean_d(input, size, byteIncrement);

    for (i = 0; i < size; i++)
    {
        dev = (*((double*)ptr)) - ave;
        sum += dev * dev;
        ptr += byteIncrement;
    }

    return sum /= size;
}


f_t variance_int32(int32_t *input, int size, int byteIncrement)
{
	int i;
	f_t dev;
	f_t sum = 0;
	f_t ave;
	char *ptr = (char*)input;

	// Validate size
	if (size <= 0)
		return 0;

	ave = mean_int32(input, size, byteIncrement);

	for (i = 0; i < size; i++)
	{
		dev = ((f_t)(*((int32_t*)ptr))) - ave;
		sum += dev * dev;
		ptr += byteIncrement;
	}

	return sum /= size;
}


double variance_int64(int64_t *input, int size, int byteIncrement)
{
	int i;
	double dev;
	double sum = 0;
	double ave;
	char *ptr = (char*)input;

	// Validate size
	if (size <= 0)
		return 0;

	ave = mean_int64(input, size, byteIncrement);

	for (i = 0; i < size; i++)
	{
		dev = ((double)(*((int64_t*)ptr))) - ave;
		sum += dev * dev;
		ptr += byteIncrement;
	}

	return sum /= size;
}


f_t variance_mean( f_t *input, f_t *ave, int size, int byteIncrement )
{
	int i;
	f_t dev;
	f_t sum = 0;
	char *ptr = (char*)input;
	
	// Validate size
	if (size <= 0)
	{
		return 0;
	}

    *ave = mean(input, size, byteIncrement);

	for(i=0; i<size; i++)
	{
		dev = (*((f_t*)ptr)) - *ave;
		sum += dev * dev;
		ptr += byteIncrement;
	}
	
	return sum /= size;
}


f_t standard_deviation( f_t *input, int size, int byteIncrement )
{
    // Validate size
	if (size <= 0)
	{
		return 0;
	}

	return _SQRT( variance( input, size, byteIncrement ) );
}

double standard_deviation_d(double *input, int size, int byteIncrement)
{
    // Validate size
    if (size <= 0)
        return 0;

    return sqrt(variance_d(input, size, byteIncrement));
}


f_t standard_deviation_int32(int32_t *input, int size, int byteIncrement)
{
	// Validate size
	if (size <= 0)
	{
		return 0;
	}

	return _SQRT(variance_int32(input, size, byteIncrement));
}


double standard_deviation_int64(int64_t *input, int size, int byteIncrement)
{
	// Validate size
	if (size <= 0)
	{
		return 0;
	}

	return sqrt(variance_int64(input, size, byteIncrement));
}


f_t standard_deviation_mean( f_t *input, f_t *mean, int size, int byteIncrement )
{
	// Validate size
	if (size <= 0)
	{
		return 0;
	}

	return _SQRT( variance_mean( input, mean, size, byteIncrement ) );
}


void standard_deviation_Vec3( ixVector3 result, ixVector3 input, int size, int byteIncrement )
{
	// Validate size
	if (size <= 0)
	{
		return;
	}
		
	result[0] = _SQRT( variance( &input[0], size, byteIncrement ) );
	result[1] = _SQRT( variance( &input[1], size, byteIncrement ) );
	result[2] = _SQRT( variance( &input[2], size, byteIncrement ) );
}


void stardard_deviation_mean_Vec3( ixVector3 result, ixVector3 input, ixVector3 mean, int size, int byteIncrement )
{
	// Validate size
	if (size <= 0)
	{
		return;
	}
		
	result[0] = _SQRT( variance_mean( &input[0], &mean[0], size, byteIncrement ) );
	result[1] = _SQRT( variance_mean( &input[1], &mean[1], size, byteIncrement ) );
	result[2] = _SQRT( variance_mean( &input[2], &mean[2], size, byteIncrement ) );
}


void mean_Vec3( ixVector3 ave, ixVector3 input, int size, int byteIncrement )
{
	ave[0] = mean(&input[0], size, byteIncrement);
	ave[1] = mean(&input[1], size, byteIncrement);
	ave[2] = mean(&input[2], size, byteIncrement);
}


// Initialize Alpha Filter alpha and beta values
void init_realtime_std_dev_Vec3( sRTSDVec3 *s, float dt, float aveCornerFreqHz, float varCornerFreqHz, ixVector3 initVal )
{
	memset( s, 0, sizeof(sRTSDVec3) );
	cpy_Vec3_Vec3( s->ave, initVal );

// 	f_t dc;
// 	dc = dt * cornerFreqHz;
// 	s->alpha  = dc / (1.0f + dc);
// 	s->beta   = 1.0f - s->alpha;

	lpf_alpha_beta( dt, aveCornerFreqHz, &s->aveAlph, &s->aveBeta );
	lpf_alpha_beta( dt, varCornerFreqHz, &s->varAlph, &s->varBeta );
}


// This does not produce as accurate of a standard deviation and is more processing expensive because it must run every
// iteration whereas standard_deviation_Vec3() does not.  It does require less memory.
void realtime_std_dev_Vec3( f_t *input, sRTSDVec3 *s )
{
	// Mean - LPF of input
	O0_LPF_Vec3( s->ave, input, s->aveAlph, s->aveBeta );

	// Variance
	ixVector3 dev;
	ixVector3 var;
	sub_Vec3_Vec3( dev, input, s->ave );
	mul_Vec3_Vec3( var, dev, dev );
	O0_LPF_Vec3( s->var, var, s->varAlph, s->varBeta );
	
	// Standard Deviation
	sqrt_Vec3( s->std, s->var );
}


f_t root_mean_squared(f_t *input, int size, int byteIncrement, float ave)
{
	// Validate size
	if (size <= 0)
		return 0;

	return _SQRT(delta_mean(input, size, byteIncrement, ave));
}


f_t delta_mean(f_t *input, int size, int byteIncrement, float ave)
{
	int i;
	f_t dev;
	f_t sum = 0;
	char *ptr = (char*)input;

	// Validate size
	if (size <= 0)
		return 0;

	for (i = 0; i < size; i++)
	{
		dev = (*((f_t*)ptr)) - ave;
		sum += dev * dev;
		ptr += byteIncrement;
	}

	return sum /= size;
}
