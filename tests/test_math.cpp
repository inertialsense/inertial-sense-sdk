#include <gtest/gtest.h>
#include <stdlib.h>
#include <cmath>
#include "../ISConstants.h"
#include "../ISMatrix.h"

// #include "Eigen/Core"
// #include "Eigen/Dense"

ixVector3 random_vectors[25] = {
  {-0.0376278050814f, 0.471775699711f, -0.336572370974f},
  {0.842139998851f, -0.113277302409f, -0.435361598132f},
  {0.402876930871f, -0.998517068538f, 0.956603957591f},
  {0.366004030077f, -0.966554559399f, 0.236455814495f},
  {0.170963581611f, -0.892193316086f, -0.360102936987f},
  {-0.675191763273f, -0.794118513048f, 0.561367212903f},
  {-0.0299477253533f, 0.0938177650483f, 0.525814272724f},
  {-0.676191678521f, -0.0780862208203f, -0.272955681219f},
  {-0.435749833209f, -0.673810649938f, -0.896559097382f},
  {0.709083915552f, -0.135067363969f, -0.385492450532f},
  {-0.38728558039f, -0.502219301225f, 0.323557018529f},
  {-0.186870345154f, 0.554827454101f, 0.921567682061f},
  {-0.142106787605f, -0.764876359963f, 0.00303689980819f},
  {-0.677798963582f, -0.664595954482f, 0.339274533414f},
  {-0.700464041114f, 0.325731535871f, -0.621492014391f},
  {-0.604865828708f, 0.270639620454f, 0.188624833185f},
  {0.464205180183f, -0.461504601245f, -0.578708441515f},
  {0.498899172115f, -0.582342366402f, -0.694758083436f},
  {0.0710544604541f, -0.63603887083f, -0.521799692437f},
  {-0.372025413205f, 0.83531212357f, 0.232484576742f},
  {0.790872496361f, -0.89600683592f, 0.783984438621f},
  {0.236462609786f, -0.636362560394f, 0.203951290805f},
  {0.831924307534f, -0.482532468579f, 0.0600026189612f},
  {0.0562194856302f, -0.605799189029f, -0.556494338297f},
  {-0.85014432598f, 0.0632157037573f, 0.0272188414114f} };

#define REQUIRE_SUPER_CLOSE(x, y) EXPECT_TRUE(std::fabs(x - y) < 1e-6)

#define REQUIRE_SORTA_CLOSE(x, y) EXPECT_TRUE(std::fabs(x - y) < 1e-3)

#define VEC3_CLOSE(x, y)	REQUIRE_SUPER_CLOSE(x[0], y(0)); \
							REQUIRE_SUPER_CLOSE(x[1], y(1)); \
							REQUIRE_SUPER_CLOSE(x[2], y(2))

#define MAT3_CLOSE(x, y)	REQUIRE_SUPER_CLOSE(x[0], y(0, 0));\
							REQUIRE_SUPER_CLOSE(x[1], y(0, 1));\
							REQUIRE_SUPER_CLOSE(x[2], y(0, 2));\
							REQUIRE_SUPER_CLOSE(x[3], y(1, 0));\
							REQUIRE_SUPER_CLOSE(x[4], y(1, 1));\
							REQUIRE_SUPER_CLOSE(x[5], y(1, 2));\
							REQUIRE_SUPER_CLOSE(x[6], y(2, 0));\
							REQUIRE_SUPER_CLOSE(x[7], y(2, 1));\
							REQUIRE_SUPER_CLOSE(x[8], y(2, 2))

#define MAT3_SORTA_CLOSE(x, y)	REQUIRE_SORTA_CLOSE(x[0], y(0, 0));\
								REQUIRE_SORTA_CLOSE(x[1], y(0, 1));\
								REQUIRE_SORTA_CLOSE(x[2], y(0, 2));\
								REQUIRE_SORTA_CLOSE(x[3], y(1, 0));\
								REQUIRE_SORTA_CLOSE(x[4], y(1, 1));\
								REQUIRE_SORTA_CLOSE(x[5], y(1, 2));\
								REQUIRE_SORTA_CLOSE(x[6], y(2, 0));\
								REQUIRE_SORTA_CLOSE(x[7], y(2, 1));\
								REQUIRE_SORTA_CLOSE(x[8], y(2, 2))


bool testVectors()
{
	// First test dot product functions
	for (int i = 0; i < 24; i++)
	{
		// 3D Vector operations
		ixVector3 IS_v1 = { random_vectors[i][0], random_vectors[i][1], random_vectors[i][2] };
		ixVector3 IS_v2 = { random_vectors[i + 1][0], random_vectors[i + 1][1], random_vectors[i + 1][2] };

#if 0
		Eigen::Vector3f eig_v1, eig_v2;
		eig_v1 << random_vectors[i][0], random_vectors[i][1], random_vectors[i][2];
		eig_v2 << random_vectors[i + 1][0], random_vectors[i + 1][1], random_vectors[i + 1][2];

		// Dot Product
		REQUIRE_SUPER_CLOSE(dot_Vec3_Vec3(IS_v1, IS_v2), eig_v1.dot(eig_v2));

		// Cross Product
		Vector3 crossed;
		Eigen::Vector3f eig_crossed = eig_v1.cross(eig_v2);
		cross_Vec3(crossed, IS_v1, IS_v2);
		VEC3_CLOSE(crossed, eig_crossed);

		// Scalar Multiply
		Vector3 multiplied;
		mul_Vec3_X(multiplied, IS_v1, 3.287);
		Eigen::Vector3f eig_multiplied = 3.287 * eig_v1;
		VEC3_CLOSE(multiplied, eig_multiplied);

		// Add and Subtract
		Vector3 added;
		Vector3 subtracted;
		add_Vec3_Vec3(added, IS_v1, IS_v2);
		sub_Vec3_Vec3(subtracted, IS_v1, IS_v2);
		Eigen::Vector3f eig_added = eig_v1 + eig_v2;
		Eigen::Vector3f eig_subtracted = eig_v1 - eig_v2;
		VEC3_CLOSE(added, eig_added);
		VEC3_CLOSE(subtracted, eig_subtracted);

		// Normalize
		Vector3 IS_v1_normalized;
		normalize_Vec3(IS_v1_normalized, IS_v1);
		Eigen::Vector3f eig_v1_normalized = eig_v1;
		eig_v1_normalized.normalize();
		VEC3_CLOSE(IS_v1_normalized, eig_v1_normalized);

		// Norm
		f_t IS_v1_norm = mag_Vec3(IS_v1);
		double eig_v1_norm = eig_v1.norm();
		REQUIRE_SUPER_CLOSE(IS_v1_norm, eig_v1_norm);
#endif
	}
	return true;
}

bool testMatrixOperations()
{
	for (int i = 0; i < 25; i++)
	{
		int j = (i + 3) % 25;
		int k = (i + 6) % 25;
		ixVector3 IS_vec1 = { random_vectors[i][0], random_vectors[i][1], random_vectors[i][2] };
		ixMatrix3 IS_mat1 = { random_vectors[j][0], random_vectors[j][1], random_vectors[j][2],
							random_vectors[(j + 1) % 25][0], random_vectors[(j + 1) % 25][1], random_vectors[(j + 1) % 25][2],
							random_vectors[(j + 2) % 25][0], random_vectors[(j + 2) % 25][1], random_vectors[(j + 2) % 25][2] };
		ixMatrix3 IS_mat2 = { random_vectors[k][0], random_vectors[k][1], random_vectors[k][2],
							random_vectors[(k + 1) % 25][0], random_vectors[(k + 1) % 25][1], random_vectors[(k + 1) % 25][2],
							random_vectors[(k + 2) % 25][0], random_vectors[(k + 2) % 25][1], random_vectors[(k + 2) % 25][2] };

#if 0
		Eigen::Vector3f eig_vec1;
		eig_vec1 << random_vectors[i][0], random_vectors[i][1], random_vectors[i][2];
		Eigen::Matrix3f eig_mat1;
		eig_mat1 << random_vectors[j][0], random_vectors[j][1], random_vectors[j][2],
			random_vectors[(j + 1) % 25][0], random_vectors[(j + 1) % 25][1], random_vectors[(j + 1) % 25][2],
			random_vectors[(j + 2) % 25][0], random_vectors[(j + 2) % 25][1], random_vectors[(j + 2) % 25][2];
		Eigen::Matrix3f eig_mat2;
		eig_mat2 << random_vectors[k][0], random_vectors[k][1], random_vectors[k][2],
			random_vectors[(k + 1) % 25][0], random_vectors[(k + 1) % 25][1], random_vectors[(k + 1) % 25][2],
			random_vectors[(k + 2) % 25][0], random_vectors[(k + 2) % 25][1], random_vectors[(k + 2) % 25][2];

		// Transpose
		ixMatrix3 IS_mat1_T;
		transpose_Mat3(IS_mat1_T, IS_mat1);
		Eigen::Matrix3f eig_mat1_T = eig_mat1.transpose();
		MAT3_CLOSE(IS_mat1_T, eig_mat1_T);

		// Matrix Multiply
		ixMatrix3 IS_mat1_mat_mult;
		mul_Mat3x3_Mat3x3(IS_mat1_mat_mult, IS_mat1, IS_mat2);
		Eigen::Matrix3f eig_mat1_mat_mult = eig_mat1 * eig_mat2;
		MAT3_CLOSE(IS_mat1_mat_mult, eig_mat1_mat_mult);

		// Vector/Matrix Multiply
		Vector3 IS_vec_mat_mult;
		mul_Mat3x3_Vec3x1(IS_vec_mat_mult, IS_mat1, IS_vec1);
		Eigen::Vector3f eig_vec_mat_mult = eig_mat1 * eig_vec1;
		VEC3_CLOSE(IS_vec_mat_mult, eig_vec_mat_mult);

		// Inverse
		ixMatrix3 IS_mat1_inv;
		inv_Mat3(IS_mat1_inv, IS_mat1);
		Eigen::Matrix3f eig_mat1_inv = eig_mat1.inverse();
		MAT3_SORTA_CLOSE(IS_mat1_inv, eig_mat1_inv);
#endif
	}
	return true;
}


TEST(Math_Vector3_Operations, math_vector3_oper)
{
	testVectors();
}

TEST(Math_ixMatrix3_Operations, math_matrix3_oper)
{
	testMatrixOperations();
}
