#include <linear_algebra/LinearAlgebra.h>
#include <Eigen/Dense>
#include <cmath>

namespace linear_algebra {

/**
 * This function should return the vector (2, 1, 3) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorA() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (2, 1, 3).
	result<< 2,1,3;

	return result;
}

/**
 * This function should return the vector (-1, -5, 2) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorB() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (-1, -5, 2).
	result<< -1,-5,2;

	return result;
}


Eigen::Matrix3d LinearAlgebra::matrixM() {
	Eigen::Matrix3d result;

	result(0,0) = 1;
	result(0,1) = 2;
	result(0,2) = 7;
	result(1,0) = 0;
	result(1,1) = 2;
	result(1,2) = 0;
	result(2,0) = 1;
	result(2,1) = 0;
	result(2,2) = -1;


	return result;
}

Eigen::Matrix3d LinearAlgebra::invMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	result = M.inverse();

	return result;
}

Eigen::Matrix3d LinearAlgebra::transposeMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	// TODO: return the transpose of matrix M
	result = M.transpose();

	return result;
}

double LinearAlgebra::detOfMatrixM(const Eigen::Matrix3d& M)
{
	double result = 0.0;

	// TODO: return the determinant of matrix M
	result = M.determinant();

	return result;
}

double LinearAlgebra::dotProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	double result = 0.0;

	// TODO: return the dot product of vectors a and b.
	result = a.dot(b);

	return result;
}


bool LinearAlgebra::isLinearIndependent(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	bool result = false;

	/* TODO: test if the vectors a and b are linear independent.
	   Return true if they are independent, false if they are dependent.*/

	// Eigen::Vector3d norm1;
	// Eigen::Vector3d norm2;
	// norm1 = a.normalized();
	// norm2 = b.normalized();
	// if (norm1 != norm2)
	// 	result = true;

	Eigen::Vector3d c = a.cross(b);

	double value = c.norm();

	if(value == 0)
		return false;
	else
		return true;


}

Eigen::Vector3d LinearAlgebra::solveLinearSystem(const Eigen::Matrix3d& M, const Eigen::Vector3d& a) {
	Eigen::Vector3d result;

	// TODO: Solve Mx = a for x and return x.
	Eigen::Matrix3d mInverse = M.inverse();

	result = mInverse * a;

	return result;
}

}  // namespace linear_algebra
