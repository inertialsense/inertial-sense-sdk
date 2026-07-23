/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISPolynomial.h
 * @brief Least-squares polynomial curve fitting and Horner's-method polynomial evaluation.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_POLYNOMIAL_H_
#define IS_POLYNOMIAL_H_

// C API...
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Fit a polynomial of the given order to a set of (x, y) samples via least squares.
 *
 * Solves c = inv(At A) At y, where A is the Vandermonde-like design matrix built from x. For
 * example, for a 2nd-order fit (num_coefs = 2):
 *
 *     y    =     A       c
 * [ y1 ] = [ x1  1 ] [ c0 ]
 * [ y2 ]   [ x2  1 ] [ c1 ]
 * [ yn ]   [ xn  1 ]
 * (n x 1)   (n x 2)  (2 x 1)
 *
 * and for a 3rd-order fit (num_coefs = 3):
 *
 *     y    =        A          c
 * [ y1 ] = [ x1^2  x1  1 ] [ c0 ]
 * [ y2 ]   [ x2^2  x2  1 ] [ c1 ]
 * [ yn ]   [ xn^2  xn  1 ] [ c2 ]
 * (n x 1)      (n x 3)     (3 x 1)
 *
 * @param n         Number of samples in x and y.
 * @param x         Input data array (independent variable), length n.
 * @param y         Input data array (dependent variable), length n.
 * @param coef      Output: polynomial coefficients, length num_coefs.
 * @param num_coefs 1 + the highest degree (order) of the monomials with non-zero coefficients.
 * @return 0 on success, -1 on failure.
 */
char ixPolyFit(const int n, const float x[], const float y[], float coef[], const int num_coefs);

/**
 * @brief Evaluate a polynomial y = f(x) at x using Horner's method.
 *
 * The polynomial has degree = coef_size-1. The coefficients are multipliers of monomial terms of
 * decreasing order, i.e. the polynomial is assumed to be written in the form:
 *
 *     y(x) = coef[0]*x^n + coef[1]*x^(n-1) + ... + coef[n-1]*x + coef[n]
 *
 * where n is the order (coef_size-1) of the polynomial and the largest index in coef is n.
 *
 * @param coef_size Number of elements in coef (degree + 1).
 * @param coef      Polynomial coefficients, highest-order term first, length coef_size.
 * @param x         Value at which to evaluate the polynomial.
 * @return y = f(x), the evaluated polynomial value.
 */
float ixPolyHorner(const int coef_size, const float coef[], const float x);


#ifdef __cplusplus
} // extern C
#endif

#endif // IS_POLYNOMIAL_H_
