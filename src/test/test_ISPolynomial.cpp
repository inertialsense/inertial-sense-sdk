#include <gtest/gtest.h>

#include "../ISMatrix.h"
#include "../ISPolynomial.h"


#define REQUIRE_SUPER_CLOSE(x, y) EXPECT_TRUE(std::fabs(x - y) < 1e-6)
#define REQUIRE_SORTA_CLOSE(x, y) EXPECT_TRUE(std::fabs(x - y) < 1e-3)


TEST(ISMatrix, matrix_mul_A3x5_B5x3)
{
#undef M_SIZE
#undef N_SIZE
#define M_SIZE	3
#define N_SIZE	5

	float A[M_SIZE * N_SIZE] =
	{
          1,  2,  3,  4,  5,
          6,  7,  8,  9, 10,
         11, 12, 13, 14, 15
	};

	float B[N_SIZE * M_SIZE] =
	{
          4,  5,  6,
          7,  8,  9,
         10, 11, 12,
         13, 14, 15,
         16, 17, 18
	};

	float C[M_SIZE * M_SIZE] =
	{
        180, 195, 210,
        430, 470, 510,
        680, 745, 810,
	};

	float result[M_SIZE * M_SIZE];

	// C[mxm] = A[mxn] * B[nxm]
	mul_MatMxN(result, A, B, M_SIZE, N_SIZE, M_SIZE, 0, 0, 0);

	for (int i = 0; i < M_SIZE*M_SIZE; i++)
	{
		REQUIRE_SUPER_CLOSE(result[i], C[i]);
	}
}


TEST(ISMatrix, matrix_mul_At3x5_Bt5x3)
{
#undef M_SIZE
#undef N_SIZE
#define M_SIZE	3
#define N_SIZE	5

	float A[M_SIZE * N_SIZE] =
	{
		  1,  2,  3,  4,  5,
		  6,  7,  8,  9, 10,
		 11, 12, 13, 14, 15
	};

	float B[N_SIZE * M_SIZE] =
	{
		  4,  5,  6,
		  7,  8,  9,
		 10, 11, 12,
		 13, 14, 15,
		 16, 17, 18
	};

	float C[N_SIZE * N_SIZE] =
	{
         100, 154, 208, 262, 316,
         115, 178, 241, 304, 367,
         130, 202, 274, 346, 418,
         145, 226, 307, 388, 469,
         160, 250, 340, 430, 520 
	};

	float result[N_SIZE * N_SIZE];

	// C[nxn] = At[nxm] * Bt[mxn]
	mul_MatMxN(result, A, B, N_SIZE, M_SIZE, N_SIZE, 1, 1, 0);

	for (int i = 0; i < N_SIZE * N_SIZE; i++)
	{
		REQUIRE_SUPER_CLOSE(result[i], C[i]);
	}
}


TEST(ISMatrix, matrix_mul_A3x3_Bt3x3)
{
#undef M_SIZE
#undef N_SIZE
#define M_SIZE	3
#define N_SIZE	M_SIZE

	float A[M_SIZE * N_SIZE] =
	{
		  1,  2,  3,  
		  4,  5,  6,  
		  7,  8,  9,
	};

	float B[N_SIZE * M_SIZE] =
	{
		  4,  5,  6,
		  7,  8,  9,
		 10, 11, 12,
	};

	float C[M_SIZE * M_SIZE] =
	{
          32,  50,  68,
          77, 122, 167,
         122, 194, 266
	};

	float result[M_SIZE * M_SIZE];

	// C[mxm] = A[mxm] * Bt[mxm]
	mul_MatMxN(result, A, B, M_SIZE, N_SIZE, M_SIZE, 0, 1, 0);

	for (int i = 0; i < M_SIZE * M_SIZE; i++)
	{
		REQUIRE_SUPER_CLOSE(result[i], C[i]);
	}
}


TEST(ISMatrix, matrix_A_plus_mul_A3x3_Bt3x3)
{
#undef M_SIZE
#undef N_SIZE
#define M_SIZE	3
#define N_SIZE	M_SIZE

	float A[M_SIZE * N_SIZE] =
	{
		  1,  2,  3,
		  4,  5,  6,
		  7,  8,  9,
	};

	float B[N_SIZE * M_SIZE] =
	{
		  4,  5,  6,
		  7,  8,  9,
		 10, 11, 12,
	};

	float C[M_SIZE * M_SIZE] =
	{
          33,  52,  71,
          81, 127, 173,
         129, 202, 275, 
	};

	float result[M_SIZE * M_SIZE];
	for (int i = 0; i < M_SIZE * M_SIZE; i++)
	{
		result[i] = A[i];
	}

	// C[mxm] = A[mxm] * Bt[mxm]
	mul_MatMxN(result, A, B, M_SIZE, N_SIZE, M_SIZE, 0, 1, 1);

	for (int i = 0; i < M_SIZE * M_SIZE; i++)
	{
		REQUIRE_SUPER_CLOSE(result[i], C[i]);
	}
}


TEST(ISPolynomial, ixPolyHorner)
{
#undef N_COEF
#undef N_SAMPLES
#define N_COEF      3	// 2nd order poly
#define N_SAMPLES	4

	float coef[N_COEF] = { 5.6f, 1.2f, 3.4f };	// y = c[0]*x^2 + c[1]*x + c[2]
	float x[N_SAMPLES] = { 0, 1, 2, 3 };
	float expected[N_SAMPLES] = { 3.4f, 10.2f, 28.2f, 57.4f };
	float result[N_SAMPLES];

	for (int i = 0; i < N_SAMPLES; i++)
	{
		result[i] = ixPolyHorner(N_COEF, coef, x[i]);
		REQUIRE_SORTA_CLOSE(expected[i], result[i]);
	}
}


TEST(ISPolynomial, ixPolyFit_1st_order)
{
#undef N_COEF
#undef N_SAMPLES
#define N_COEF      2	// 1st order poly
#define N_SAMPLES	4

	float coef[N_COEF] = { 1.2f, 3.4f };	// y = c[0]*x + c[1]
	float x[N_SAMPLES] = { 0, 1, 2, 3 };
	float y[N_SAMPLES] = { 0 };
	float result[N_COEF];

	for (int i = 0; i < N_SAMPLES; i++)
	{
		y[i] = ixPolyHorner(N_COEF, coef, x[i]);
	}

	EXPECT_TRUE(ixPolyFit(N_SAMPLES, x, y, result, N_COEF) == 0);

	for (int i = 0; i < N_COEF; i++)
	{
		REQUIRE_SORTA_CLOSE(coef[i], result[i]);
	}
}


TEST(ISPolynomial, ixPolyFit_2nd_order)
{
#undef N_COEF
#undef N_SAMPLES
#define N_COEF      3	// 1st order poly
#define N_SAMPLES	5

	float coef[N_COEF] = { 1.2f, 3.4f, 5.6f };	// y = c[0]*x^2 + c[1]*x + c[2]
	float x[N_SAMPLES] = { 0, 1, 2, 3, 4 };
	float y[N_SAMPLES] = { 0 };
	float result[N_COEF];

	for (int i = 0; i < N_SAMPLES; i++)
	{
		y[i] = ixPolyHorner(N_COEF, coef, x[i]);
	}

	EXPECT_TRUE(ixPolyFit(N_SAMPLES, x, y, result, N_COEF) == 0);

	for (int i = 0; i < N_COEF; i++)
	{
		REQUIRE_SORTA_CLOSE(coef[i], result[i]);
	}
}

