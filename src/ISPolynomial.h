/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_POLYNOMIAL_H_
#define IS_POLYNOMIAL_H_

// C API...
#ifdef __cplusplus
extern "C" {
#endif


// Curve fit polynomial to x,y data.
// Returns 1 on success, -1 on failure
char ixPolyFit(const int n, const float x[], const float y[], float coef[], const int num_coef);

// ----------------------------------------------------------------------------
//  y = horner(degree, coef, x) evaluates a polynomial y = f(x) at x.  The polynomial has
//                    degree = n-1.  The coefficients of the polynomial are stored
//                    in the 1-D array c, which has n elements.
//
//  NOTE:  The polynomial coefficients are multipliers of monomial terms of
//         decreasing order.  In other words, the polynomial is assumed to be
//         written in the form
//
//            y = coef[0]*x^n + coef[1]*x^(n-1) + ... + coef[n-1]*x + coef[n]
//
//         where n is the order (coef_size-1) of the polynomial and the largest index in coef is n.
float ixPolyHorner(const int coef_size, const float coef[], const float x);


#ifdef __cplusplus
} // extern C
#endif

#endif // IS_POLYNOMIAL_H_
