#include <windows-helpers.h>
#include <projective_geometry/ProjectiveGeometry.h>
#include <iostream>
#include <cmath>
using namespace projective_geometry;
int main(int /* argc */, char ** /*argv*/)
{
	const Eigen::Vector3d x1(1,0,2);
	const Eigen::Vector3d x2(2,3,0);
	
	const CameraParameters param = ProjectiveGeometry::setCameraParameters(4.0 * ProjectiveGeometry::PI / 180.0);

	const Eigen::Matrix3d calibrationMatrix = ProjectiveGeometry::calibrationMatrix(param);
	std::cout << "The calibrationMatrix is: "<< std::endl  << calibrationMatrix << std::endl << std::endl;

	const Eigen::MatrixXd projectionMatrix = ProjectiveGeometry::projectionMatrix(calibrationMatrix, param);
	std::cout << "The projectionMatrix is: "<< std::endl  << projectionMatrix << std::endl << std::endl;
	
	std::cout << "The projection of x1 is: "<< std::endl  << ProjectiveGeometry::projectPoint(x1,projectionMatrix) << std::endl << std::endl;
	std::cout << "The projection of x2 is: "<< std::endl  << ProjectiveGeometry::projectPoint(x2,projectionMatrix) << std::endl << std::endl;
	
	const Eigen::Vector2d imagePoint(240, 180);
	const double tableHeight = 0.12;
	std::cout << "The reprojection of pixel (" << imagePoint.transpose() << ") onto the table in front of the robot is: " << std::endl;
	std::cout << ProjectiveGeometry::reprojectTo3D(imagePoint, calibrationMatrix, param, tableHeight) << std::endl << std::endl;

    wait();
	return 0;
}
