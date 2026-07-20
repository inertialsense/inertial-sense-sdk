/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <stdlib.h>
#include "ISMatrix.h"
#include "data_sets.h"

void LU(const f_t *M, i_t n, f_t *L, f_t *U);
char solve_upper(f_t *result, i_t n, f_t *A, f_t *b);
char solve_lower(f_t *result, i_t n, f_t *A, f_t *b);

/**
 * @brief Multiplies an MxN matrix A by an NxP matrix B, producing an MxP result; A and/or B may be treated as transposed without physically transposing them, and the product can overwrite, add to, or subtract from the existing contents of result.
 * @note Skips multiply-accumulate terms where either operand element is zero. This is a sparse-matrix optimization, not a change to the mathematical result.
 *
 * @param result - output MxP matrix (row-major), overwritten/added-to/subtracted-from depending on add
 * @param A - input matrix, MxN normally, or NxM if transpose_A is set (row-major)
 * @param B - input matrix, NxP normally, or PxN if transpose_B is set (row-major)
 * @param m - number of rows in A (and result)
 * @param n - number of columns in A / rows in B (the shared dimension)
 * @param p - number of columns in B (and result)
 * @param transpose_A - nonzero to treat A as transposed (stored NxM, used as MxN)
 * @param transpose_B - nonzero to treat B as transposed (stored PxN, used as NxP)
 * @param add - 0 to overwrite result, >0 to add the product into result, <0 to subtract the product from result
 */
void mul_MatMxN(f_t *result, const f_t *A, const f_t *B, i_t m, i_t n, i_t p, char transpose_A, char transpose_B, char add)
{
    i_t i;
    i_t j;
    i_t k;

    for (i = 0; i < m; i++)
    {
        f_t * O_i = result + i * p;

        for (j = 0; j < p; j++)
        {
            f_t s = 0.0f;
            f_t * O_i_j = O_i + j;

            for (k = 0; k < n; k++)
            {
                const f_t * a;
                const f_t * b;

                if (transpose_A)
                    a = A + k * m + i;
                else
                    a = A + i * n + k;

                if (is_zero(a))
                    continue;

                if (transpose_B)
                    b = B + j * n + k;
                else
                    b = B + k * p + j;

                if (is_zero(b))
                    continue;

                s += *a * *b;
            }

            if (add == 0)
                *O_i_j = s;
            else if (add > 0)
                *O_i_j += s;
            else
                *O_i_j -= s;
        }
    }
}

#if 0
void addNxM(void * result, const void * A_ptr, const void * B_ptr, i_t m, i_t n, char transpose_B)
{
    i_t i;
    i_t j;
    i_t k;

    f_t * OUT = result;
    const f_t * A = A_ptr;
    const f_t * B = B_ptr;

    for (i = 0; i < m; i++)
    {
        const f_t * A_i = A + i * n;
        f_t * O_i = OUT + i * p;

        for (j = 0; j < p; j++)
        {
//             f_t s = 0;
            f_t * O_i_j = O_i + j;


            for (k = 0; k < n; k++)
            {
                const f_t * a = A_i + k;
                const f_t * b;

//                 if (is_zero(a))
//                     continue;

                if (transpose_B)
                    b = B + j * n + k;
                else
                    b = B + k * p + j;

                if (is_zero(b))
                    continue;

                s += *a * *b;
            }

            *O_i_j = s;
        }
    }
}
#endif


/**
 * @brief Sets an NxN matrix to the identity matrix (zeros with 1s on the diagonal).
 *
 * @param A - output NxN matrix (row-major)
 * @param n - matrix dimension (rows and columns)
 */
void eye_MatN(f_t *A, i_t n)
{
    zero_MatMxN(A, n, n);

    // Set diagonals to 1
    for (int i=0; i < n; i++)
        A[i*n + i] = 1.0f;
}


// Compute the LU factorization of the square matrix A
/**
 * @brief Computes the LU decomposition of an NxN matrix M (Gaussian elimination without pivoting), producing lower-triangular L (unit diagonal) and upper-triangular U such that M = L*U.
 * @note Allocates a temporary NxN scratch matrix internally and silently returns without writing L/U if that allocation fails. No pivoting is performed, so this can be numerically unstable or fail (divide-by-zero) for matrices with a zero or near-zero pivot.
 *
 * @param M - input NxN matrix to decompose (row-major)
 * @param n - matrix dimension (rows and columns)
 * @param L - output NxN lower-triangular matrix with unit diagonal (row-major)
 * @param U - output NxN upper-triangular matrix (row-major)
 */
void LU(const f_t *M, i_t n, f_t *L, f_t *U)
{
    int in, kn;

    f_t *A = (f_t*)MALLOC(sizeof(f_t)*n*n);
    if (A == 0) { return; }

    cpy_MatMxN(A, M, n, n);

    for (int k=0; k < n - 1; k++)
    {
        for (int i=k + 1; i < n; i++)
        {
            in = i*n;
            kn = k*n;
//             f_t * Ai = A + i*n;

            A[in + k] = A[in + k] / A[kn + k];

            for (int j=k + 1; j < n; j++)
            {
#ifdef NO_FPU
                T &        A_i_j(A[in + j]);
                const T &    A_i_k(A[in + k]);
                const T &    A_k_j(A[kn + j]);

                if (is_zero(A_i_k)
                    || is_zero(A_k_j)
                  )
                    continue;

                A_i_j -= A_i_k * A_k_j;
#else
                A[in + j] -= A[in + k] * A[kn + j];
#endif
            }
        }
    }

    eye_MatN(L, n);

    /* Separate the L matrix */
    for (int j=0; j < n - 1; j++)
        for (int i=j + 1; i < n; i++)
            L[i*n + j] = A[i*n + j];

    /* Separate the M matrix */
    zero_MatMxN(U, n, n);

    for (int i=0; i < n; i++)
        for (int j=i; j < n; j++)
            U[i*n + j] = A[i*n + j];
            
    FREE(A);
}

// Return 0 on success, -1 on numerical error
/**
 * @brief Solves the upper-triangular linear system A*x = b for x, via back substitution.
 * @note Only the upper triangle of A (row i, columns i..n-1) is referenced. result is written with a stride of n (result[i*n] holds the i-th unknown), matching how inv_MatN() uses this to fill one column of an inverse matrix at a time.
 *
 * @param result - output solution vector, written with stride n (result[i*n] holds row i's value)
 * @param n - matrix dimension (rows and columns)
 * @param A - upper-triangular NxN coefficient matrix (row-major)
 * @param b - right-hand-side vector, one entry per row
 *
 * @return 0 on success, -1 if a zero pivot (A[i][i] == 0) is encountered
 */
char solve_upper(f_t *result, i_t n, f_t *A, f_t *b)
{
    for (int i=n - 1; i >= 0; i--)
    {
        f_t s = b[i];

        // Reference a row
        f_t *A_i = &A[i*n];

        for (int j=i + 1; j < n; ++j)
        {
#ifdef NO_FPU
            const T &    A_i_j(A_i[j]);
            const T &    x_j(x[j]);

            if (is_zero(A_i_j) || is_zero(x_j))
                continue;

            s -= A_i_j * x_j;
#else
            s -= A_i[j] * result[j*n];
#endif
        }

        // Prevent divide by zero
        if (A_i[i]==0.0f)
            return -1;
        
        result[i*n] = s / A_i[i];
    }
    
    return 0;
}

// Return 0 on success, -1 on numerical error
/**
 * @brief Solves the lower-triangular linear system A*x = b for x, via forward substitution.
 * @note Only the lower triangle of A (row i, columns 0..i, including the diagonal) is referenced. result is written with a stride of n (result[i*n] holds the i-th unknown), matching how inv_MatN() uses this to fill one column of an inverse matrix at a time.
 *
 * @param result - output solution vector, written with stride n (result[i*n] holds row i's value)
 * @param n - matrix dimension (rows and columns)
 * @param A - lower-triangular NxN coefficient matrix (row-major)
 * @param b - right-hand-side vector, one entry per row
 *
 * @return 0 on success, -1 if a zero pivot (A[i][i] == 0) is encountered
 */
char solve_lower(f_t *result, i_t n, f_t *A, f_t *b)
{
    for (int i=0; i < n; ++i)
    {
        f_t s = b[i];

        // Reference a row
        f_t *A_i = &A[i*n];

        for (int j=0; j < i; ++j)
        {
#ifdef NO_FPU
            const T &    A_i_j(A_i[j]);
            const T &    x_j(x[j]);

            if (is_zero(A_i_j)|| is_zero(x_j))
                continue;

            s -= A_i_j * x_j;
#else
            s -= A_i[j] * result[j*n];
#endif
        }

        // Prevent divide by zero
        if (A_i[i]==0.0f)
            return -1;

        result[i*n] = s / A_i[i];
    }
    
    return 0;
}


// Return 0 on success, -1 on numerical error
/**
 * @brief Computes the inverse of an NxN matrix M via LU decomposition: factors M = L*U, inverts U and L separately by solving one identity column at a time, then multiplies invU * invL to form the inverse.
 * @note Allocates several NxN scratch matrices on the heap and busy-waits (empty while loop) if any allocation fails, rather than returning an error - callers should ensure sufficient heap is available.
 *
 * @param result - output NxN inverse matrix (row-major)
 * @param M - input NxN matrix to invert (row-major)
 * @param n - matrix dimension (rows and columns)
 *
 * @return 0 on success, -1 if M is singular (a zero pivot was encountered)
 */
char inv_MatN(f_t *result, const f_t *M, i_t n)
{
    char error        = 0;

    f_t *L              = (f_t*)MALLOC(sizeof(f_t)*n*n);
    f_t *U              = (f_t*)MALLOC(sizeof(f_t)*n*n);
    f_t *invL           = (f_t*)MALLOC(sizeof(f_t)*n*n);
    f_t *invU           = (f_t*)MALLOC(sizeof(f_t)*n*n);
    f_t    *identCol    = (f_t*)MALLOC(sizeof(f_t)*n);
    
    while (L==NULL || U==NULL || invL==NULL || invU==NULL || identCol==NULL) { /* Error check malloc */ }
    
    memset(identCol, 0, sizeof(f_t)*n);

    LU(M, n, L, U);

    for (int i=0; i < n; i++)
    {
        identCol[i] = 1;
        if (solve_upper(&invU[i], n, U, identCol) || // Fill a column
            solve_lower(&invL[i], n, L, identCol))
        {        
            error = -1;        
            break;
        }
        identCol[i] = 0;
    }

    // result = invU * invL
    if (!error)
        mul_MatMxN(result, invU, invL, n, n, n, 0, 0, 0);

    FREE(L);
    FREE(U);
    FREE(invL);
    FREE(invU);
    FREE(identCol);

    return error;
}


/**
 * @brief Transposes an MxN matrix M into an NxM result.
 *
 * @param result - output NxM transposed matrix (row-major)
 * @param M - input MxN matrix to transpose (row-major)
 * @param m - number of rows in M
 * @param n - number of columns in M
 */
void trans_MatMxN(f_t *result, const f_t *M, int m, int n)
{
    i_t i;
    i_t j;

    const f_t * A = (const f_t*)M;

    for (i = 0; i < m; i++)
    {
        f_t * O_i = result + i;

        for (j = 0; j < n; j++)
        {
            // Copy value
            *O_i = *A;

            // Increment pointers
            A++;
            O_i += m;
        }
    }
}


/**
 * @brief Multiplies two 3x3 matrices: result = m1 * m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - left-hand 3x3 matrix (row-major)
 * @param m2 - right-hand 3x3 matrix (row-major)
 */
void mul_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2)
{
    // Row 1
    result[0] = m1[0]*m2[0] + m1[1]*m2[3] + m1[2]*m2[6];
    result[1] = m1[0]*m2[1] + m1[1]*m2[4] + m1[2]*m2[7];
    result[2] = m1[0]*m2[2] + m1[1]*m2[5] + m1[2]*m2[8];
    // Row 2
    result[3] = m1[3]*m2[0] + m1[4]*m2[3] + m1[5]*m2[6];
    result[4] = m1[3]*m2[1] + m1[4]*m2[4] + m1[5]*m2[7];
    result[5] = m1[3]*m2[2] + m1[4]*m2[5] + m1[5]*m2[8];
    // Row 3
    result[6] = m1[6]*m2[0] + m1[7]*m2[3] + m1[8]*m2[6];
    result[7] = m1[6]*m2[1] + m1[7]*m2[4] + m1[8]*m2[7];
    result[8] = m1[6]*m2[2] + m1[7]*m2[5] + m1[8]*m2[8];
}

/**
 * @brief Double-precision variant of mul_Mat3x3_Mat3x3(): multiplies two 3x3 matrices, result = m1 * m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - left-hand 3x3 matrix (row-major)
 * @param m2 - right-hand 3x3 matrix (row-major)
 */
void mul_Mat3x3_Mat3x3_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6];
    result[1] = m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7];
    result[2] = m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8];
    // Row 2
    result[3] = m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6];
    result[4] = m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7];
    result[5] = m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8];
    // Row 3
    result[6] = m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6];
    result[7] = m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7];
    result[8] = m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8];
}

/**
 * @brief Multiplies the transpose of a 3x3 matrix by another 3x3 matrix: result = m1^T * m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - 3x3 matrix that is implicitly transposed before multiplying (row-major)
 * @param m2 - right-hand 3x3 matrix (row-major)
 */
void mul_Mat3x3_Trans_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2)
{
    // Row 1
    result[0] = m1[0]*m2[0] + m1[3]*m2[3] + m1[6]*m2[6];
    result[1] = m1[0]*m2[1] + m1[3]*m2[4] + m1[6]*m2[7];
    result[2] = m1[0]*m2[2] + m1[3]*m2[5] + m1[6]*m2[8];
    // Row 2
    result[3] = m1[1]*m2[0] + m1[4]*m2[3] + m1[7]*m2[6];
    result[4] = m1[1]*m2[1] + m1[4]*m2[4] + m1[7]*m2[7];
    result[5] = m1[1]*m2[2] + m1[4]*m2[5] + m1[7]*m2[8];
    // Row 3
    result[6] = m1[2]*m2[0] + m1[5]*m2[3] + m1[8]*m2[6];
    result[7] = m1[2]*m2[1] + m1[5]*m2[4] + m1[8]*m2[7];
    result[8] = m1[2]*m2[2] + m1[5]*m2[5] + m1[8]*m2[8];
}

/**
 * @brief Double-precision variant of mul_Mat3x3_Trans_Mat3x3(): result = m1^T * m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - 3x3 matrix that is implicitly transposed before multiplying (row-major)
 * @param m2 - right-hand 3x3 matrix (row-major)
 */
void mul_Mat3x3_Trans_Mat3x3_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[3] * m2[3] + m1[6] * m2[6];
    result[1] = m1[0] * m2[1] + m1[3] * m2[4] + m1[6] * m2[7];
    result[2] = m1[0] * m2[2] + m1[3] * m2[5] + m1[6] * m2[8];
    // Row 2
    result[3] = m1[1] * m2[0] + m1[4] * m2[3] + m1[7] * m2[6];
    result[4] = m1[1] * m2[1] + m1[4] * m2[4] + m1[7] * m2[7];
    result[5] = m1[1] * m2[2] + m1[4] * m2[5] + m1[7] * m2[8];
    // Row 3
    result[6] = m1[2] * m2[0] + m1[5] * m2[3] + m1[8] * m2[6];
    result[7] = m1[2] * m2[1] + m1[5] * m2[4] + m1[8] * m2[7];
    result[8] = m1[2] * m2[2] + m1[5] * m2[5] + m1[8] * m2[8];
}

/**
 * @brief Multiplies a 3x3 matrix by the transpose of another 3x3 matrix: result = m1 * m2^T.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - left-hand 3x3 matrix (row-major)
 * @param m2 - 3x3 matrix that is implicitly transposed before multiplying (row-major)
 */
void mul_Mat3x3_Mat3x3_Trans(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2)
{
    // Row 1
    result[0] = m1[0]*m2[0] + m1[1]*m2[1] + m1[2]*m2[2];
    result[1] = m1[0]*m2[3] + m1[1]*m2[4] + m1[2]*m2[5];
    result[2] = m1[0]*m2[6] + m1[1]*m2[7] + m1[2]*m2[8];
    // Row 2
    result[3] = m1[3]*m2[0] + m1[4]*m2[1] + m1[5]*m2[2];
    result[4] = m1[3]*m2[3] + m1[4]*m2[4] + m1[5]*m2[5];
    result[5] = m1[3]*m2[6] + m1[4]*m2[7] + m1[5]*m2[8];
    // Row 3
    result[6] = m1[6]*m2[0] + m1[7]*m2[1] + m1[8]*m2[2];
    result[7] = m1[6]*m2[3] + m1[7]*m2[4] + m1[8]*m2[5];
    result[8] = m1[6]*m2[6] + m1[7]*m2[7] + m1[8]*m2[8];
}

/**
 * @brief Double-precision variant of mul_Mat3x3_Mat3x3_Trans(): result = m1 * m2^T.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - left-hand 3x3 matrix (row-major)
 * @param m2 - 3x3 matrix that is implicitly transposed before multiplying (row-major)
 */
void mul_Mat3x3_Mat3x3_Trans_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2];
    result[1] = m1[0] * m2[3] + m1[1] * m2[4] + m1[2] * m2[5];
    result[2] = m1[0] * m2[6] + m1[1] * m2[7] + m1[2] * m2[8];
    // Row 2
    result[3] = m1[3] * m2[0] + m1[4] * m2[1] + m1[5] * m2[2];
    result[4] = m1[3] * m2[3] + m1[4] * m2[4] + m1[5] * m2[5];
    result[5] = m1[3] * m2[6] + m1[4] * m2[7] + m1[5] * m2[8];
    // Row 3
    result[6] = m1[6] * m2[0] + m1[7] * m2[1] + m1[8] * m2[2];
    result[7] = m1[6] * m2[3] + m1[7] * m2[4] + m1[8] * m2[5];
    result[8] = m1[6] * m2[6] + m1[7] * m2[7] + m1[8] * m2[8];
}

/**
 * @brief Adds two 3x3 matrices element-wise: result = m1 + m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - first 3x3 matrix (row-major)
 * @param m2 - second 3x3 matrix (row-major)
 */
void add_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2)
{
    // Row 1
    result[0] = m1[0] + m2[0];
    result[1] = m1[1] + m2[1];
    result[2] = m1[2] + m2[2];
    // Row 2
    result[3] = m1[3] + m2[3];
    result[4] = m1[4] + m2[4];
    result[5] = m1[5] + m2[5];
    // Row 3
    result[6] = m1[6] + m2[6];
    result[7] = m1[7] + m2[7];
    result[8] = m1[8] + m2[8];
}

/**
 * @brief Subtracts one 3x3 matrix from another element-wise: result = m1 - m2.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m1 - minuend 3x3 matrix (row-major)
 * @param m2 - subtrahend 3x3 matrix (row-major)
 */
void sub_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2)
{
    // Row 1
    result[0] = m1[0] - m2[0];
    result[1] = m1[1] - m2[1];
    result[2] = m1[2] - m2[2];
    // Row 2
    result[3] = m1[3] - m2[3];
    result[4] = m1[4] - m2[4];
    result[5] = m1[5] - m2[5];
    // Row 3
    result[6] = m1[6] - m2[6];
    result[7] = m1[7] - m2[7];
    result[8] = m1[8] - m2[8];
}

/**
 * @brief Multiplies a 2x2 matrix by a 2-element column vector: result = m * v.
 *
 * @param result - output 2-element vector
 * @param m - 2x2 matrix (row-major)
 * @param v - input 2-element vector
 */
void mul_Mat2x2_Vec2x1(ixVector2 result, const ixMatrix2 m, const ixVector2 v)
{
    result[0] = m[0]*v[0] + m[1]*v[1];
    result[1] = m[2]*v[0] + m[3]*v[1];
}

/**
 * @brief Multiplies the transpose of a 2x2 matrix by a 2-element column vector: result = m^T * v.
 *
 * @param result - output 2-element vector
 * @param m - 2x2 matrix that is implicitly transposed before multiplying (row-major)
 * @param v - input 2-element vector
 */
void mul_Mat2x2_Trans_Vec2x1(ixVector2 result, const ixMatrix2 m, const ixVector2 v)
{
    result[0] = m[0]*v[0] + m[2]*v[1];
    result[1] = m[1]*v[0] + m[3]*v[1];
}

/**
 * @brief Multiplies a 3x3 matrix by a 3-element column vector: result = m * v.
 *
 * @param result - output 3-element vector
 * @param m - 3x3 matrix (row-major)
 * @param v - input 3-element vector
 */
void mul_Mat3x3_Vec3x1(ixVector3 result, const ixMatrix3 m, const ixVector3 v)
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    result[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    result[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

/**
 * @brief Multiplies the transpose of a 3x3 matrix by a 3-element column vector: result = m^T * v.
 *
 * @param result - output 3-element vector
 * @param m - 3x3 matrix that is implicitly transposed before multiplying (row-major)
 * @param v - input 3-element vector
 */
void mul_Mat3x3_Trans_Vec3x1(ixVector3 result, const ixMatrix3 m, const ixVector3 v)
{
    result[0] = m[0]*v[0] + m[3]*v[1] + m[6]*v[2];
    result[1] = m[1]*v[0] + m[4]*v[1] + m[7]*v[2];
    result[2] = m[2]*v[0] + m[5]*v[1] + m[8]*v[2];
}

/**
 * @brief Multiplies a 4x4 matrix by a 4-element column vector: result = m * v.
 *
 * @param result - output 4-element vector
 * @param m - 4x4 matrix (row-major)
 * @param v - input 4-element vector
 */
void mul_Mat4x4_Vec4x1(ixVector4 result, const ixMatrix4 m, const ixVector4 v)
{
    result[0] =  m[0]*v[0] +  m[1]*v[1] +  m[2]*v[2] +  m[3]*v[3];
    result[1] =  m[4]*v[0] +  m[5]*v[1] +  m[6]*v[2] +  m[7]*v[3];
    result[2] =  m[8]*v[0] +  m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
    result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

/**
 * @brief Multiplies the transpose of a 4x4 matrix by a 4-element column vector: result = m^T * v.
 *
 * @param result - output 4-element vector
 * @param m - 4x4 matrix that is implicitly transposed before multiplying (row-major)
 * @param v - input 4-element vector
 */
void mul_Mat4x4_Trans_Vec4x1(ixVector4 result, const ixMatrix4 m, const ixVector4 v)
{
    result[0] =  m[0]*v[0] + m[4]*v[1] +  m[8]*v[2] + m[12]*v[3];
    result[1] =  m[1]*v[0] + m[5]*v[1] +  m[9]*v[2] + m[13]*v[3];
    result[2] =  m[2]*v[0] + m[6]*v[1] + m[10]*v[2] + m[14]*v[3];
    result[3] =  m[3]*v[0] + m[7]*v[1] + m[11]*v[2] + m[15]*v[3];
}

/**
 * @brief Negates every element of a 3x3 matrix: result = -m.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m - input 3x3 matrix (row-major)
 */
void neg_Mat3x3(ixMatrix3 result, const ixMatrix3 m)
{
    // Row 1
    result[0] = -m[0];
    result[1] = -m[1];
    result[2] = -m[2];
    // Row 2
    result[3] = -m[3];
    result[4] = -m[4];
    result[5] = -m[5];
    // Row 3
    result[6] = -m[6];
    result[7] = -m[7];
    result[8] = -m[8];
}

/**
 * @brief Scales a 3x3 matrix by a scalar: result = m * x.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m - input 3x3 matrix (row-major)
 * @param x - scalar multiplier
 */
void mul_Mat3x3_X(ixMatrix3 result, const ixMatrix3 m, const f_t x)
{
    // Row 1
    result[0] = m[0] * x;
    result[1] = m[1] * x;
    result[2] = m[2] * x;
    // Row 2
    result[3] = m[3] * x;
    result[4] = m[4] * x;
    result[5] = m[5] * x;
    // Row 3
    result[6] = m[6] * x;
    result[7] = m[7] * x;
    result[8] = m[8] * x;
}

/**
 * @brief Divides every element of a 3x3 matrix by a scalar: result = m / x.
 * @note Implemented as multiplication by 1/x; does not check for x == 0.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m - input 3x3 matrix (row-major)
 * @param x - scalar divisor
 */
void div_Mat3x3_X(ixMatrix3 result, const ixMatrix3 m, const f_t x)
{
    f_t d = 1.0f / x;
    // Row 1
    result[0] = m[0] * d;
    result[1] = m[1] * d;
    result[2] = m[2] * d;
    // Row 2
    result[3] = m[3] * d;
    result[4] = m[4] * d;
    result[5] = m[5] * d;
    // Row 3
    result[6] = m[6] * d;
    result[7] = m[7] * d;
    result[8] = m[8] * d;
}

/**
 * @brief Computes the outer product of two 3-element vectors, producing a 3x3 matrix: result = v1 * v2^T.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param v1 - 3-element column vector
 * @param v2 - 3-element row vector
 */
void mul_Vec3x1_Vec1x3(ixMatrix3 result, const ixVector3 v1, const  ixVector3 v2)
{
    // Row 1
    result[0] = v1[0]*v2[0];
    result[1] = v1[0]*v2[1];
    result[2] = v1[0]*v2[2];
    // Row 2
    result[3] = v1[1]*v2[0];
    result[4] = v1[1]*v2[1];
    result[5] = v1[1]*v2[2];
    // Row 3
    result[6] = v1[2]*v2[0];
    result[7] = v1[2]*v2[1];
    result[8] = v1[2]*v2[2];
}

/**
 * @brief Multiplies two 2-element vectors element-wise (Hadamard product): result = v1 .* v2.
 *
 * @param result - output 2-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void mul_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2)
{
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[1];
}

/**
 * @brief Multiplies two 3-element vectors element-wise (Hadamard product): result = v1 .* v2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void mul_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[1];
    result[2] = v1[2] * v2[2];
}

/**
 * @brief Multiplies two 4-element vectors element-wise (Hadamard product): result = v1 .* v2.
 *
 * @param result - output 4-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void mul_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2)
{
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[1];
    result[2] = v1[2] * v2[2];
    result[3] = v1[3] * v2[3];
}

/**
 * @brief Computes the element-wise square root of a 3-element vector.
 *
 * @param result - output 3-element vector
 * @param v - input 3-element vector (each element should be non-negative)
 */
void sqrt_Vec3(ixVector3 result, const ixVector3 v)
{
    result[0] = _SQRT(v[0]);
    result[1] = _SQRT(v[1]);
    result[2] = _SQRT(v[2]);
}

/**
 * @brief Computes the element-wise square root of a 4-element vector.
 *
 * @param result - output 4-element vector
 * @param v - input 4-element vector (each element should be non-negative)
 */
void sqrt_Vec4(ixVector4 result, const ixVector4 v)
{
    result[0] = _SQRT(v[0]);
    result[1] = _SQRT(v[1]);
    result[2] = _SQRT(v[2]);
    result[3] = _SQRT(v[3]);
}

/**
 * @brief Computes the element-wise absolute value of a 2-element single-precision vector.
 *
 * @param result - output 2-element vector
 * @param v - input 2-element vector
 */
void abs_Vec2(ixVector2 result, const ixVector2 v)
{
    result[0] = _FABS(v[0]);
    result[1] = _FABS(v[1]);
}

/**
 * @brief Computes the element-wise absolute value of a 2-element double-precision vector.
 *
 * @param result - output 2-element vector
 * @param v - input 2-element vector
 */
void abs_Vec2d(ixVector2d result, const ixVector2d v)
{
    result[0] = fabs(v[0]);
    result[1] = fabs(v[1]);
}

/**
 * @brief Computes the element-wise absolute value of a 3-element single-precision vector.
 *
 * @param result - output 3-element vector
 * @param v - input 3-element vector
 */
void abs_Vec3(ixVector3 result, const ixVector3 v)
{
    result[0] = _FABS(v[0]);
    result[1] = _FABS(v[1]);
    result[2] = _FABS(v[2]);    
}

/**
 * @brief Computes the element-wise absolute value of a 3-element double-precision vector.
 *
 * @param result - output 3-element vector
 * @param v - input 3-element vector
 */
void abs_Vec3d(ixVector3d result, const ixVector3d v)
{
    result[0] = fabs(v[0]);
    result[1] = fabs(v[1]);
    result[2] = fabs(v[2]);
}

/**
 * @brief Computes the element-wise absolute value of a 4-element single-precision vector.
 *
 * @param result - output 4-element vector
 * @param v - input 4-element vector
 */
void abs_Vec4(ixVector4 result, const ixVector4 v)
{
    result[0] = _FABS(v[0]);
    result[1] = _FABS(v[1]);
    result[2] = _FABS(v[2]);
    result[3] = _FABS(v[3]);
}

/**
 * @brief Computes the element-wise absolute value of a 4-element double-precision vector.
 *
 * @param result - output 4-element vector
 * @param v - input 4-element vector
 */
void abs_Vec4d(ixVector4d result, const ixVector4d v)
{
    result[0] = fabs(v[0]);
    result[1] = fabs(v[1]);
    result[2] = fabs(v[2]);
    result[3] = fabs(v[3]);
}

/**
 * @brief Computes the dot product of a 2-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 2-element vector
 *
 * @return sum of squares of v's elements
 */
f_t dot_Vec2(const ixVector2 v)
{
    return  v[0] * v[0] +
            v[1] * v[1];
}

/**
 * @brief Computes the dot product of a 3-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 3-element vector
 *
 * @return sum of squares of v's elements
 */
f_t dot_Vec3(const ixVector3 v)
{
    return  v[0] * v[0] +
            v[1] * v[1] +
            v[2] * v[2];
}

/**
 * @brief Computes the dot product of a 4-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 4-element vector
 *
 * @return sum of squares of v's elements
 */
f_t dot_Vec4(const ixVector4 v)
{
    return  v[0] * v[0] +
            v[1] * v[1] +
            v[2] * v[2] +
            v[3] * v[3];
}

/**
 * @brief Double-precision variant: computes the dot product of a 2-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 2-element vector
 *
 * @return sum of squares of v's elements
 */
double dot_Vec2d(const ixVector2d v)
{
    return  v[0] * v[0] +
            v[1] * v[1];
}

/**
 * @brief Double-precision variant: computes the dot product of a 3-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 3-element vector
 *
 * @return sum of squares of v's elements
 */
double dot_Vec3d(const ixVector3d v)
{
    return  v[0] * v[0] +
            v[1] * v[1] +
            v[2] * v[2];
}

/**
 * @brief Double-precision variant: computes the dot product of a 4-element vector with itself (its magnitude squared): v . v.
 *
 * @param v - input 4-element vector
 *
 * @return sum of squares of v's elements
 */
double dot_Vec4d(const ixVector4d v)
{
    return  v[0] * v[0] +
            v[1] * v[1] +
            v[2] * v[2] +
            v[3] * v[3];
}

/**
 * @brief Computes the dot product of two 2-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
f_t dot_Vec2_Vec2(const ixVector2 v1, const ixVector2 v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1];
}

/**
 * @brief Computes the dot product of two 3-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
f_t dot_Vec3_Vec3(const ixVector3 v1, const ixVector3 v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1] +
            v1[2] * v2[2];
}

/**
 * @brief Computes the dot product of two 4-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
f_t dot_Vec4_Vec4(const ixVector4 v1, const ixVector4 v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1] +
            v1[2] * v2[2] +
            v1[3] * v2[3];
}

/**
 * @brief Double-precision variant: computes the dot product of two 2-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
double dot_Vec2d_Vec2d(const ixVector2d v1, const ixVector2d v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1];
}

/**
 * @brief Double-precision variant: computes the dot product of two 3-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
double dot_Vec3d_Vec3d(const ixVector3d v1, const ixVector3d v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1] +
            v1[2] * v2[2];
}

/**
 * @brief Double-precision variant: computes the dot product of two 4-element vectors: v1 . v2.
 *
 * @param v1 - first input vector
 * @param v2 - second input vector
 *
 * @return sum of the element-wise products of v1 and v2
 */
double dot_Vec4d_Vec4d(const ixVector4d v1, const ixVector4d v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1] +
            v1[2] * v2[2] +
            v1[3] * v2[3];
}

/**
 * @brief Computes the Euclidean magnitude (length) of a 2-element vector.
 *
 * @param v - input 2-element vector
 *
 * @return sqrt(v . v)
 */
f_t mag_Vec2(const ixVector2 v)
{
    return _SQRT(dot_Vec2(v));
}

/**
 * @brief Computes the Euclidean magnitude (length) of a 3-element vector.
 *
 * @param v - input 3-element vector
 *
 * @return sqrt(v . v)
 */
f_t mag_Vec3(const ixVector3 v)
{
    return _SQRT(dot_Vec3(v));
}

/**
 * @brief Computes the Euclidean magnitude (length) of a 4-element vector.
 *
 * @param v - input 4-element vector
 *
 * @return sqrt(v . v)
 */
f_t mag_Vec4(const ixVector4 v)
{
    return _SQRT(dot_Vec4(v));
}

//_______________________________________________________________________________________________
//observe that cross product output cannot overwrite cross product input without destroying logic
/**
 * @brief Computes the 3-D cross product of two vectors: result = v1 x v2.
 * @note result must not alias v1 or v2 - the computation reads all of v1/v2 while writing result, so an in-place call would produce incorrect values.
 *
 * @param result - output 3-element vector (must be distinct from v1 and v2)
 * @param v1 - first input 3-element vector
 * @param v2 - second input 3-element vector
 */
void cross_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0]; 
}

/**
 * @brief Computes the 3-D cross product of two single-precision vectors, storing the result in double precision: result = v1 x v2.
 * @note Same aliasing caveat as cross_Vec3(): result must not overwrite v1 or v2.
 *
 * @param result - output 3-element double-precision vector
 * @param v1 - first input 3-element vector
 * @param v2 - second input 3-element vector
 */
void crossd_Vec3(ixVector3d result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = (double)(v1[1] * v2[2] - v1[2] * v2[1]);
    result[1] = (double)(v1[2] * v2[0] - v1[0] * v2[2]);
    result[2] = (double)(v1[0] * v2[1] - v1[1] * v2[0]);
}

/**
 * @brief Scales a 2-element vector by a scalar: result = v * x.
 *
 * @param result - output 2-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec2_X(ixVector2 result, const ixVector2 v, const f_t x)
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
}

/**
 * @brief Double-precision variant: scales a 2-element vector by a scalar: result = v * x.
 *
 * @param result - output 2-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec2d_X(ixVector2d result, const ixVector2d v, const double x)
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
}

/**
 * @brief Scales a 3-element vector by a scalar: result = v * x.
 *
 * @param result - output 3-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec3_X(ixVector3 result, const ixVector3 v, const f_t x)
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
    result[2] = v[2]*x;
}

/**
 * @brief Double-precision variant: scales a 3-element vector by a scalar: result = v * x.
 *
 * @param result - output 3-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec3d_X(ixVector3d result, const ixVector3d v, const double x)
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
    result[2] = v[2]*x;
}

/**
 * @brief Scales a 4-element vector by a scalar: result = v * x.
 *
 * @param result - output 4-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec4_X(ixVector4 result, const ixVector4 v, const f_t x)
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
    result[2] = v[2]*x;
    result[3] = v[3]*x;
}

/**
 * @brief Double-precision variant: scales a 4-element vector by a scalar: result = v * x.
 *
 * @param result - output 4-element vector
 * @param v - input vector
 * @param x - scalar multiplier
 */
void mul_Vec4d_X(ixVector4d result, const ixVector4d v, const double x)
{
    result[0] = v[0] * x;
    result[1] = v[1] * x;
    result[2] = v[2] * x;
    result[3] = v[3] * x;
}

/**
 * @brief Divides every element of a 3-element vector by a scalar: result = v / x.
 * @note Implemented as multiplication by 1/x; does not check for x == 0.
 *
 * @param result - output 3-element vector
 * @param v - input vector
 * @param x - scalar divisor
 */
void div_Vec3_X(ixVector3 result, const ixVector3 v, const f_t x)
{
    f_t d = 1.0f / x;
    result[0] = v[0]*d;
    result[1] = v[1]*d;
    result[2] = v[2]*d;
}

/**
 * @brief Double-precision variant: divides every element of a 3-element vector by a scalar: result = v / x.
 * @note Implemented as multiplication by 1/x; does not check for x == 0.
 *
 * @param result - output 3-element vector
 * @param v - input vector
 * @param x - scalar divisor
 */
void div_Vec3d_X(ixVector3d result, const ixVector3d v, const double x)
{
    double d = 1.0 / x;
    result[0] = v[0] * d;
    result[1] = v[1] * d;
    result[2] = v[2] * d;
}

/**
 * @brief Divides every element of a 4-element vector by a scalar: result = v / x.
 * @note Implemented as multiplication by 1/x; does not check for x == 0.
 *
 * @param result - output 4-element vector
 * @param v - input vector
 * @param x - scalar divisor
 */
void div_Vec4_X(ixVector4 result, const ixVector4 v, const f_t x)
{
    f_t d = 1.0f / x;
    result[0] = v[0]*d;
    result[1] = v[1]*d;
    result[2] = v[2]*d;
    result[3] = v[3]*d;
}
/**
 * @brief Double-precision variant: divides every element of a 4-element vector by a scalar: result = v / x.
 * @note Implemented as multiplication by 1/x; does not check for x == 0.
 *
 * @param result - output 4-element vector
 * @param v - input vector
 * @param x - scalar divisor
 */
void div_Vec4d_X(ixVector4d result, const ixVector4d v, const double x)
{
    double d = 1.0 / x;
    result[0] = v[0] * d;
    result[1] = v[1] * d;
    result[2] = v[2] * d;
    result[3] = v[3] * d;
}

/**
 * @brief Adds two 2-element vectors element-wise: result = v1 + v2.
 *
 * @param result - output 2-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void add_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2)
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
}

/**
 * @brief Adds two 3-element vectors element-wise: result = v1 + v2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void add_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

/**
 * @brief Double-precision variant: adds two 3-element vectors element-wise: result = v1 + v2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void add_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2)
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

/**
 * @brief Computes a weighted sum of two 3-element vectors: result = k1*v1 + k2*v2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 * @param k1 - scalar weight applied to v1
 * @param k2 - scalar weight applied to v2
 */
void add_K1Vec3_K2Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2, float k1, float k2)
{
    result[0] = k1 * v1[0] + k2 * v2[0];
    result[1] = k1 * v1[1] + k2 * v2[1];
    result[2] = k1 * v1[2] + k2 * v2[2];
}

/**
 * @brief Adds two 4-element vectors element-wise: result = v1 + v2.
 *
 * @param result - output 4-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void add_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2)
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
    result[3] = v1[3] + v2[3];
}

/**
 * @brief Double-precision variant: adds two 4-element vectors element-wise: result = v1 + v2.
 *
 * @param result - output 4-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void add_Vec4d_Vec4d(ixVector4d result, const ixVector4d v1, const ixVector4d v2)
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
    result[3] = v1[3] + v2[3];
}

/**
 * @brief Subtracts one 3-element vector from another element-wise: result = v1 - v2.
 *
 * @param result - output 3-element vector
 * @param v1 - minuend vector
 * @param v2 - subtrahend vector
 */
void sub_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

/**
 * @brief Subtracts one 2-element vector from another element-wise: result = v1 - v2.
 *
 * @param result - output 2-element vector
 * @param v1 - minuend vector
 * @param v2 - subtrahend vector
 */
void sub_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2)
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
}

/**
 * @brief Double-precision variant: subtracts one 3-element vector from another element-wise: result = v1 - v2.
 *
 * @param result - output 3-element vector
 * @param v1 - minuend vector
 * @param v2 - subtrahend vector
 */
void sub_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2)
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

/**
 * @brief Subtracts one 4-element vector from another element-wise: result = v1 - v2.
 *
 * @param result - output 4-element vector
 * @param v1 - minuend vector
 * @param v2 - subtrahend vector
 */
void sub_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2)
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
    result[3] = v1[3] - v2[3];
}

/**
 * @brief Divides one 3-element vector by another element-wise: result = v1 ./ v2.
 * @note Does not check for zero elements in v2.
 *
 * @param result - output 3-element vector
 * @param v1 - dividend vector
 * @param v2 - divisor vector
 */
void div_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = v1[0] / v2[0];
    result[1] = v1[1] / v2[1];
    result[2] = v1[2] / v2[2];
}

/**
 * @brief Divides one 4-element vector by another element-wise: result = v1 ./ v2.
 * @note Does not check for zero elements in v2.
 *
 * @param result - output 4-element vector
 * @param v1 - dividend vector
 * @param v2 - divisor vector
 */
void div_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2)
{
    result[0] = v1[0] / v2[0];
    result[1] = v1[1] / v2[1];
    result[2] = v1[2] / v2[2];
    result[3] = v1[3] / v2[3];
}

/**
 * @brief Negates every element of a 3-element vector: result = -v.
 *
 * @param result - output 3-element vector
 * @param v - input vector
 */
void neg_Vec3(ixVector3 result, const ixVector3 v)
{
    result[0] = -v[0];
    result[1] = -v[1];
    result[2] = -v[2];
}

/**
 * @brief Computes the element-wise average (midpoint) of two 3-element vectors: result = (v1 + v2) / 2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void mean_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    add_Vec3_Vec3(result, v1, v2);
    mul_Vec3_X(result, result, 0.5f);
}

/**
 * @brief Double-precision variant: computes the element-wise average (midpoint) of two 3-element vectors: result = (v1 + v2) / 2.
 *
 * @param result - output 3-element vector
 * @param v1 - first input vector
 * @param v2 - second input vector
 */
void mean_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2)
{
    add_Vec3d_Vec3d(result, v1, v2);
    mul_Vec3d_X(result, result, 0.5);
}

/**
 * @brief Checks whether a 3x3 matrix is exactly equal to the identity matrix.
 *
 * @param m - 3x3 matrix to check (row-major)
 *
 * @return nonzero (true) if m is the identity matrix, 0 (false) otherwise
 */
int mat3x3_IsIdentity(const f_t m[])
{
    return  (m[0]==1.0f) && (m[1]==0.0f) && (m[2]==0.0f) &&
            (m[3]==0.0f) && (m[4]==1.0f) && (m[5]==0.0f) &&
            (m[6]==0.0f) && (m[7]==0.0f) && (m[8]==1.0f);
}

/**
 * @brief Copies an MxN source matrix A into a sub-block of an RxC destination matrix result, placing A's top-left corner at (r_offset, c_offset).
 * @note Silently does nothing if A would not fit within result at the given offset (m+r_offset > r or n+c_offset > c).
 *
 * @param result - output RxC matrix (row-major); the sub-block starting at (r_offset, c_offset) is overwritten
 * @param r - number of rows in result
 * @param c - number of columns in result
 * @param r_offset - row offset within result where A's copy begins
 * @param c_offset - column offset within result where A's copy begins
 * @param A - input MxN matrix to copy (row-major)
 * @param m - number of rows in A
 * @param n - number of columns in A
 */
void cpy_MatRxC_MatMxN(f_t *result, i_t r, i_t c, i_t r_offset, i_t c_offset, f_t *A, i_t m, i_t n)
{
    // Ensure source matrix A fits within result matrix
    if ((m + r_offset) > r || (n + c_offset) > c)
        return;

    // Set result pointer to first location
    result += c*r_offset + c_offset;

    int rowSize = sizeof(f_t)*n;
    for (int mi=0; mi<m; mi++)
    {
        // Copy row
        memcpy(result, A, rowSize);

        // Update to next row
        A        += n;
        result    += c;
    }
}


/**
 * @brief Transposes a 2x2 matrix: result = m^T.
 *
 * @param result - output 2x2 matrix (row-major)
 * @param m - input 2x2 matrix (row-major)
 */
void transpose_Mat2(ixMatrix2 result, const ixMatrix2 m)
{
    // Row 1
    result[0] = m[0];
    result[1] = m[2];
    // Row 2
    result[2] = m[1];
    result[3] = m[3];
}

/**
 * @brief Transposes a 3x3 matrix: result = m^T.
 *
 * @param result - output 3x3 matrix (row-major)
 * @param m - input 3x3 matrix (row-major)
 */
void transpose_Mat3(ixMatrix3 result, const ixMatrix3 m)
{
    // Row 1
    result[0] = m[0];
    result[1] = m[3];
    result[2] = m[6];
    // Row 2
    result[3] = m[1];
    result[4] = m[4];
    result[5] = m[7];
    // Row 3
    result[6] = m[2];
    result[7] = m[5];
    result[8] = m[8];
}

/**
 * @brief Transposes a 4x4 matrix: result = m^T.
 *
 * @param result - output 4x4 matrix (row-major)
 * @param m - input 4x4 matrix (row-major)
 */
void transpose_Mat4(ixMatrix4 result, const ixMatrix4 m)
{
    // Row 1
    result[ 0] = m[ 0];
    result[ 1] = m[ 4];
    result[ 2] = m[ 8];
    result[ 3] = m[12];
    // Row 2
    result[ 4] = m[ 1];
    result[ 5] = m[ 5];
    result[ 6] = m[ 9];
    result[ 7] = m[13];
    // Row 3
    result[ 8] = m[ 2];
    result[ 9] = m[ 6];
    result[10] = m[10];
    result[11] = m[14];
    // Row 4
    result[12] = m[ 3];
    result[13] = m[ 7];
    result[14] = m[11];
    result[15] = m[15];
}

/**
 * @brief Computes the inverse of a 2x2 matrix using the closed-form adjugate/determinant formula.
 *
 * @param result - output 2x2 inverse matrix (row-major)
 * @param m - input 2x2 matrix to invert (row-major)
 *
 * @return 1 on success, -1 if m is singular (determinant is 0)
 */
char inv_Mat2(ixMatrix2 result, ixMatrix2 m)
{
    f_t invDet, det = m[0] * m[3] - m[1] * m[2];

    if (det!=0.0f)
        invDet = 1.0f/det;
    else
        return -1;

    // Row 1
    result[0] = m[3] * (invDet);
    result[1] = m[1] * (-invDet);
    // Row 2
    result[2] = m[2] * (-invDet);
    result[3] = m[0] * (invDet);

    return 1;
}

/**
 * @brief Computes the inverse of a 3x3 matrix using the closed-form adjugate/determinant (cofactor expansion) formula.
 *
 * @param result - output 3x3 inverse matrix (row-major)
 * @param m - input 3x3 matrix to invert (row-major)
 *
 * @return 0 on success, -1 if m is singular (determinant is 0)
 */
char inv_Mat3(ixMatrix3 result, const ixMatrix3 m)
{
    //     | m[0] m[1] m[2] |-1             |   m[8]m[4]-m[7]m[5]  -(m[8]m[1]-m[7]m[2])   m[5]m[1]-m[4]m[2]  |
    //     | m[3] m[4] m[5] |    =  1/det * | -(m[8]m[3]-m[6]m[5])   m[8]m[0]-m[6]m[2]  -(m[5]m[0]-m[3]m[2]) |
    //     | m[6] m[7] m[8] |               |   m[7]m[3]-m[6]m[4]  -(m[7]m[0]-m[6]m[1])   m[4]m[0]-m[3]m[1]  |
    
    f_t invDet, det = m[0]*(m[8]*m[4]-m[7]*m[5]) - m[3]*(m[8]*m[1]-m[7]*m[2]) + m[6]*(m[5]*m[1]-m[4]*m[2]);

    if (det!=0)
        invDet = 1.0f / det;
    else
        return -1;
        
    // Row 1
    result[0] = invDet * (m[8]*m[4]-m[7]*m[5]);
    result[1] = invDet * (m[7]*m[2]-m[8]*m[1]);  
    result[2] = invDet * (m[5]*m[1]-m[4]*m[2]);
    // Row 2
    result[3] = invDet * (m[6]*m[5]-m[8]*m[3]);
    result[4] = invDet * (m[8]*m[0]-m[6]*m[2]); 
    result[5] = invDet * (m[3]*m[2]-m[5]*m[0]);
    // Row 3
    result[6] = invDet * (m[7]*m[3]-m[6]*m[4]);
    result[7] = invDet * (m[6]*m[1]-m[7]*m[0]);
    result[8] = invDet * (m[4]*m[0]-m[3]*m[1]);
    
    return 0;
}

/**
 * @brief Computes the inverse of a 4x4 matrix using the closed-form adjugate/determinant (cofactor expansion) formula.
 *
 * @param result - output 4x4 inverse matrix (row-major)
 * @param m - input 4x4 matrix to invert (row-major)
 *
 * @return 0 on success, -1 if m is singular (determinant is 0)
 */
char inv_Mat4(ixMatrix4 result, const ixMatrix4 m)
{
    f_t inv[16], det;
    int i;

    inv[0] = 
        m[5]  * m[10] * m[15] - 
        m[5]  * m[11] * m[14] - 
        m[9]  * m[6]  * m[15] + 
        m[9]  * m[7]  * m[14] +
        m[13] * m[6]  * m[11] - 
        m[13] * m[7]  * m[10];

    inv[4] = 
       -m[4]  * m[10] * m[15] + 
        m[4]  * m[11] * m[14] + 
        m[8]  * m[6]  * m[15] - 
        m[8]  * m[7]  * m[14] - 
        m[12] * m[6]  * m[11] + 
        m[12] * m[7]  * m[10];

    inv[8] = 
        m[4]  * m[9] * m[15] - 
        m[4]  * m[11] * m[13] - 
        m[8]  * m[5] * m[15] + 
        m[8]  * m[7] * m[13] + 
        m[12] * m[5] * m[11] - 
        m[12] * m[7] * m[9];

    inv[12] = 
       -m[4]  * m[9] * m[14] + 
        m[4]  * m[10] * m[13] +
        m[8]  * m[5] * m[14] - 
        m[8]  * m[6] * m[13] - 
        m[12] * m[5] * m[10] + 
        m[12] * m[6] * m[9];

    inv[1] = 
       -m[1]  * m[10] * m[15] + 
        m[1]  * m[11] * m[14] + 
        m[9]  * m[2] * m[15] - 
        m[9]  * m[3] * m[14] - 
        m[13] * m[2] * m[11] + 
        m[13] * m[3] * m[10];

    inv[5] = 
        m[0]  * m[10] * m[15] - 
        m[0]  * m[11] * m[14] - 
        m[8]  * m[2] * m[15] + 
        m[8]  * m[3] * m[14] + 
        m[12] * m[2] * m[11] - 
        m[12] * m[3] * m[10];

    inv[9] = 
       -m[0]  * m[9] * m[15] + 
        m[0]  * m[11] * m[13] + 
        m[8]  * m[1] * m[15] - 
        m[8]  * m[3] * m[13] - 
        m[12] * m[1] * m[11] + 
        m[12] * m[3] * m[9];

    inv[13] = 
        m[0]  * m[9] * m[14] - 
        m[0]  * m[10] * m[13] - 
        m[8]  * m[1] * m[14] + 
        m[8]  * m[2] * m[13] + 
        m[12] * m[1] * m[10] - 
        m[12] * m[2] * m[9];

    inv[2] = 
        m[1]  * m[6] * m[15] - 
        m[1]  * m[7] * m[14] - 
        m[5]  * m[2] * m[15] + 
        m[5]  * m[3] * m[14] + 
        m[13] * m[2] * m[7] - 
        m[13] * m[3] * m[6];

    inv[6] = 
       -m[0]  * m[6] * m[15] + 
        m[0]  * m[7] * m[14] + 
        m[4]  * m[2] * m[15] - 
        m[4]  * m[3] * m[14] - 
        m[12] * m[2] * m[7] + 
        m[12] * m[3] * m[6];

    inv[10] = 
        m[0]  * m[5] * m[15] - 
        m[0]  * m[7] * m[13] - 
        m[4]  * m[1] * m[15] + 
        m[4]  * m[3] * m[13] + 
        m[12] * m[1] * m[7] - 
        m[12] * m[3] * m[5];

    inv[14] = 
       -m[0]  * m[5] * m[14] + 
        m[0]  * m[6] * m[13] + 
        m[4]  * m[1] * m[14] - 
        m[4]  * m[2] * m[13] - 
        m[12] * m[1] * m[6] + 
        m[12] * m[2] * m[5];

    inv[3] = 
       -m[1] * m[6] * m[11] + 
        m[1] * m[7] * m[10] + 
        m[5] * m[2] * m[11] - 
        m[5] * m[3] * m[10] - 
        m[9] * m[2] * m[7] + 
        m[9] * m[3] * m[6];

    inv[7] = 
        m[0] * m[6] * m[11] - 
        m[0] * m[7] * m[10] - 
        m[4] * m[2] * m[11] + 
        m[4] * m[3] * m[10] + 
        m[8] * m[2] * m[7] - 
        m[8] * m[3] * m[6];

    inv[11] = 
       -m[0] * m[5] * m[11] + 
        m[0] * m[7] * m[9] + 
        m[4] * m[1] * m[11] - 
        m[4] * m[3] * m[9] - 
        m[8] * m[1] * m[7] + 
        m[8] * m[3] * m[5];

    inv[15] = 
        m[0] * m[5] * m[10] - 
        m[0] * m[6] * m[9] - 
        m[4] * m[1] * m[10] + 
        m[4] * m[2] * m[9] + 
        m[8] * m[1] * m[6] - 
        m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return -1;

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        result[i] = inv[i] * det;

    return 0;
}


/**
 * @brief Checks whether every element of an array is strictly less than a threshold value.
 *
 * @param a - array of values to check
 * @param x - threshold value
 * @param size - number of elements in a
 *
 * @return 1 if all elements are less than x, 0 if any element is greater than or equal to x
 */
int isAllLessThanX_array(f_t *a, f_t x, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (a[i] >= x) return 0;
    }
    return 1;
}

/**
 * @brief Checks whether every element of an array is strictly greater than a threshold value.
 *
 * @param a - array of values to check
 * @param x - threshold value
 * @param size - number of elements in a
 *
 * @return 1 if all elements are greater than x, 0 if any element is less than or equal to x
 */
int isAllMoreThanX_array(f_t *a, f_t x, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (a[i] <= x) return 0;
    }
    return 1;
}

/**
 * @brief Checks whether the absolute value of every element of an array is strictly less than a threshold value.
 *
 * @param a - array of values to check
 * @param x - threshold value
 * @param size - number of elements in a
 *
 * @return 1 if |a[i]| < x for all elements, 0 if any element's absolute value is greater than or equal to x
 */
int isAllAbsLessThanX_array(f_t *a, f_t x, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (_FABS(a[i]) >= x) return 0;
    }
    return 1;
}


// Initialize Alpha Filter alpha and beta values
/**
 * @brief Initializes a zeroth-order (single-pole) low-pass alpha filter's state: seeds the filtered value with initVal and derives the alpha/beta blend coefficients from the sample period and corner frequency.
 *
 * @param lpf - filter state to initialize (output)
 * @param dt - sample period in seconds
 * @param cornerFreqHz - filter corner (cutoff) frequency in Hz
 * @param initVal - initial 3-element filtered value to seed the filter state with
 */
void LPFO0_init_Vec3(sLpfO0 *lpf, f_t dt, f_t cornerFreqHz, const ixVector3 initVal)
{
    f_t dc;

    memset(lpf, 0, sizeof(sLpfO0));
    cpy_Vec3_Vec3(lpf->v, initVal);

    dc = dt * cornerFreqHz;
    lpf->alpha  = dc / (1.0f + dc);
    lpf->beta   = 1.0f - lpf->alpha;
}

// Low-Pass Alpha Filter
/**
 * @brief Applies one update step of a zeroth-order (single-pole) low-pass alpha filter: v[n+1] = beta*v[n] + alpha*input.
 *
 * @param lpf - filter state to update in place (holds current value and alpha/beta coefficients set by LPFO0_init_Vec3())
 * @param input - new 3-element input sample
 */
void LPFO0_Vec3(sLpfO0 *lpf, const ixVector3 input)
{
    // v[n+1] = beta*v[n] + alpha*input
    O0_LPF_Vec3(lpf->v, input, lpf->alpha, lpf->beta);
}



