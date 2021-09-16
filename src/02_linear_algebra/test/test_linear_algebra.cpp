#include <gtest/gtest.h>
#include <linear_algebra/LinearAlgebra.h>
#include <Eigen/Dense>
#include <cstdlib>


using namespace linear_algebra;

const double tolerance = 1e-5;

TEST(LinearAlgebra, vectorA) {
	ASSERT_NEAR(2.0, (double) LinearAlgebra::vectorA()[0], tolerance);
	ASSERT_NEAR(1.0, (double) LinearAlgebra::vectorA()[1], tolerance);
	ASSERT_NEAR(3.0, (double) LinearAlgebra::vectorA()[2], tolerance);
}

TEST(LinearAlgebra, vectorB) {
	ASSERT_NEAR(-1.0, (double) LinearAlgebra::vectorB()[0], tolerance);
	ASSERT_NEAR(-5.0, (double) LinearAlgebra::vectorB()[1], tolerance);
	ASSERT_NEAR( 2.0, (double) LinearAlgebra::vectorB()[2], tolerance);
}

TEST(LinearAlgebra, matrixM) {
	ASSERT_NEAR(1.0, (double) LinearAlgebra::matrixM()(0,0), tolerance);
	ASSERT_NEAR(2.0, (double) LinearAlgebra::matrixM()(0,1), tolerance);
	ASSERT_NEAR(7.0, (double) LinearAlgebra::matrixM()(0,2), tolerance);
	ASSERT_NEAR(0.0, (double) LinearAlgebra::matrixM()(1,0), tolerance);
	ASSERT_NEAR(2.0, (double) LinearAlgebra::matrixM()(1,1), tolerance);
	ASSERT_NEAR(0.0, (double) LinearAlgebra::matrixM()(1,2), tolerance);
	ASSERT_NEAR(1.0, (double) LinearAlgebra::matrixM()(2,0), tolerance);
	ASSERT_NEAR(0.0, (double) LinearAlgebra::matrixM()(2,1), tolerance);
	ASSERT_NEAR(-1.0, (double) LinearAlgebra::matrixM()(2,2), tolerance);
}

TEST(LinearAlgebra, invMatrixM) {
	Eigen::Matrix3d M;
	 M<< 1 , 2 ,3 
	    , 2 , 4 , 7
    	    , 2 , 5, 7;
	Eigen::Matrix3d invM;
	invM<< 7, -1, -2,
	       0, -1,  1,
	      -2,  1,  0;
	Eigen::Matrix3d result = LinearAlgebra::invMatrixM(M);
	
	ASSERT_NEAR((double)invM(0,0), (double) result(0,0), tolerance);
	ASSERT_NEAR((double)invM(0,1), (double) result(0,1), tolerance);
	ASSERT_NEAR((double)invM(0,2), (double) result(0,2), tolerance);
	ASSERT_NEAR((double)invM(1,0), (double) result(1,0), tolerance);
	ASSERT_NEAR((double)invM(1,1), (double) result(1,1), tolerance);
	ASSERT_NEAR((double)invM(1,2), (double) result(1,2), tolerance);
	ASSERT_NEAR((double)invM(2,0), (double) result(2,0), tolerance);
	ASSERT_NEAR((double)invM(2,1), (double) result(2,1), tolerance);
	ASSERT_NEAR((double)invM(2,2), (double) result(2,2), tolerance);
}

TEST(LinearAlgebra, transposeMatrixM) {
	Eigen::Matrix3d M;
	 M<< 1 , 2 ,3 
	    , 2 , 4 , 7
    	    , 2 , 5, 7;
	Eigen::Matrix3d transM;
	transM<< 1 , 2, 2,
		2, 4, 5,
		3, 7, 7;
	Eigen::Matrix3d result = LinearAlgebra::transposeMatrixM(M);
	
	ASSERT_NEAR((double)transM(0,0), (double) result(0,0), tolerance);
	ASSERT_NEAR((double)transM(0,1), (double) result(0,1), tolerance);
	ASSERT_NEAR((double)transM(0,2), (double) result(0,2), tolerance);
	ASSERT_NEAR((double)transM(1,0), (double) result(1,0), tolerance);
	ASSERT_NEAR((double)transM(1,1), (double) result(1,1), tolerance);
	ASSERT_NEAR((double)transM(1,2), (double) result(1,2), tolerance);
	ASSERT_NEAR((double)transM(2,0), (double) result(2,0), tolerance);
	ASSERT_NEAR((double)transM(2,1), (double) result(2,1), tolerance);
	ASSERT_NEAR((double)transM(2,2), (double) result(2,2), tolerance);
}

TEST(LinearAlgebra, detOfMatrixM) {
	Eigen::Matrix3d M;
	 M<< 1 , 2 ,3 
	    , 2 , 4 , 7
    	    , 2 , 5, 7;
	const double result = -1;
	ASSERT_NEAR(result, LinearAlgebra::detOfMatrixM(M), tolerance);
}

TEST(LinearAlgebra, dotProduct) {
	for (size_t i = 0; i < 3; ++i) {
		const Eigen::Vector3d a = Eigen::Vector3d::Random(), b = Eigen::Vector3d::Random();
		const double result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		ASSERT_NEAR(result, LinearAlgebra::dotProduct(a, b), tolerance);
	}
}

TEST(LinearAlgebra, linearIndependent) {
	Eigen::Vector3d vector_2_4_7, vector_0_2_5;
	vector_2_4_7 << 2, 4, 7;
	vector_0_2_5 << 0, 2, 5;
	const Eigen::Vector3d vector_zero = Eigen::Vector3d::Zero();

	ASSERT_FALSE(LinearAlgebra::isLinearIndependent(vector_2_4_7, vector_2_4_7 * 2.0));
	ASSERT_FALSE(LinearAlgebra::isLinearIndependent(vector_0_2_5, vector_0_2_5 * (-7.0)));

	ASSERT_TRUE(LinearAlgebra::isLinearIndependent(vector_2_4_7, vector_0_2_5));

	ASSERT_FALSE(LinearAlgebra::isLinearIndependent(vector_2_4_7, vector_zero));
	ASSERT_FALSE(LinearAlgebra::isLinearIndependent(vector_zero, vector_2_4_7));
}

TEST(LinearAlgebra, solveLinearSystem) {
	const Eigen::Matrix3d M = Eigen::Matrix3d::Random();
	const Eigen::Vector3d x = Eigen::Vector3d::Random();
	const Eigen::Vector3d a = M * x;

	ASSERT_NEAR((double) x[0], (double) LinearAlgebra::solveLinearSystem(M, a)[0], tolerance);
	ASSERT_NEAR((double) x[1], (double) LinearAlgebra::solveLinearSystem(M, a)[1], tolerance);
	ASSERT_NEAR((double) x[2], (double) LinearAlgebra::solveLinearSystem(M, a)[2], tolerance);
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
