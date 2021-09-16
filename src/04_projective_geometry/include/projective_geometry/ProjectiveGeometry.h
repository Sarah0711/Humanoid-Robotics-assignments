#ifndef PROJECTIVE_GEOMETRY_H_
#define PROJECTIVE_GEOMETRY_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace projective_geometry 
{
struct CameraParameters{
	double xH;
	double yH;
	double m;
	double c;

	Eigen::Vector3d X0;
	double rotX;
	double rotY;
	double rotZ;
};

class ProjectiveGeometry 
{
public:
	static const double PI;
	ProjectiveGeometry() {};
	virtual ~ProjectiveGeometry() {};
	

	static Eigen::Vector4d euclideanToHomogeneous(const Eigen::Vector3d& point);
	static Eigen::Vector2d homogeneousToEuclidean(const Eigen::Vector3d& point);
	static CameraParameters setCameraParameters(const double alpha);
	static Eigen::Matrix3d calibrationMatrix(const CameraParameters& param);
	static Eigen::MatrixXd projectionMatrix(const Eigen::Matrix3d& calibrationMatrix, const CameraParameters& param);
	static Eigen::Vector2d projectPoint(const Eigen::Vector3d& point, const Eigen::MatrixXd& projectionMatrix);
	static Eigen::Vector3d reprojectTo3D(const Eigen::Vector2d& imagePoint, const Eigen::Matrix3d& calibrationMatrix, const CameraParameters& param, const double tableHeight);

};

}  // namespace projective_geometry

#endif  // PROJECTIVE_GEOMETRY_H_
