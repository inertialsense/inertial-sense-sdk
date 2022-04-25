/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <math.h>
#include "../../SDK/src/ISMatrix.h"
#include "ISPolynomial.h"


/** ixPolyFit() polynomial curve fit
n = number of samples
x = input data array
y = output data array
coef = polynomial coefficients
order = the highest degree (or order) of the monomials with non-zero coefficients

	2nd Order Example:
		y    =     A       c
	[ y1 ] = [ x1  1 ] [ c0 ]
	[ y2 ]   [ X2  1 ] [ c1 ]
	[ yn ]   [ x2  1 ]
	(n x 1)   (n x 2)  (2 x 1)

	c = inv(At A) At y

	3rd Order Example:
		y    =        A          c
	[ y1 ] = [ x1^2  x1  1 ] [ c0 ]
	[ y2 ]   [ x2^2  X2  1 ] [ c1 ]
	[ yn ]   [ xn^2  x2  1 ] [ c2 ]
	(n x 1)      (n x 3)     (3 x 1)

	c = inv(At A) At y

@return 0 on success, -1 on failure
*/
char ixPolyFit(const int n, const float x[], const float y[], float coef[], const int num_coef)
{
#define MAX_NUM_COEF		3
#define MAX_NUMBER_SAMPLES  10

	if (num_coef < 2 || num_coef > MAX_NUM_COEF ||
        n < num_coef || n > MAX_NUMBER_SAMPLES)
		return -1;

	int p = num_coef;

#define P_MAX	(MAX_NUM_COEF)

    float A[P_MAX * MAX_NUMBER_SAMPLES];
    float AtA[P_MAX * P_MAX];     // At A
    float iAtA[P_MAX * P_MAX];    // inv(A)
	float Aty[P_MAX];

	// Populate A matrix
	switch (num_coef)
	{
	case 2:	// 1st order 
		for (int i = 0; i < n; i++)
		{
			A[i*p] = x[i];
			A[i*p + 1] = 1.0f;
		}
		break;
	case 3: // 2nd order
		for (int i = 0; i < n; i++)
		{
			A[i*p] = x[i] * x[i];
			A[i*p + 1] = x[i];
			A[i*p + 2] = 1.0f;
		}
		break;
	}

	// AtA[pxp] = At[pxn] * A[nxp]
	mul_MatMxN(AtA, A, A, p, n, p, 1, 0, 0);

	// iAtA[pxp] = (AtA[pxp])^-1
	switch (num_coef)
	{
	case 2: inv_Mat2(iAtA, AtA); break;		// 1st order
	case 3: inv_Mat3(iAtA, AtA); break;		// 2nd order
	}

	// Aty[px1] = At[pxn] * y[nx1]
	mul_MatMxN(Aty, A, y, p, n, 1, 1, 0, 0);

	// coef[px1] = (At A)^-1 At y =  iAtA[pxp] Aty[px1]
	mul_MatMxN(coef, iAtA, Aty, p, p, 1, 0, 0, 0);

	return 0;
}


float ixPolyHorner(const int coef_size, const float coef[], const float x) 
{
	float y;

	y = coef[0];
	for (int i = 1; i < coef_size; i++)
	{
		y = y*x + coef[i];
	}
	return y;
}
