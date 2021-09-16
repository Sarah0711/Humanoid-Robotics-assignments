#include <projective_geometry/ProjectiveGeometry.h>
#include <math.h>
#include <iostream>
namespace projective_geometry 
{
const double ProjectiveGeometry::PI = 3.141592654;

/**
 * \brief Converts a 3D Euclidean coordinates to homogeneous coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \return 3D point in homogeneous coordinates.
 */
Eigen::Vector4d ProjectiveGeometry::euclideanToHomogeneous(const Eigen::Vector3d& point)
{
    Eigen::Vector4d result = Eigen::Vector4d::Zero();
    //TODO
	result(0) = point(0);
	result(1) = point(1);
	result(2) = point(2);
	result(3) = 1;
    return result;
}
/**
 * \brief Converts a 2D point in homogeneous coordinates into Euclidean coordinates.
 * \param[in] 2D point in homogeneous coordinates.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::homogeneousToEuclidean(const Eigen::Vector3d& point)
{
    Eigen::Vector2d result = Eigen::Vector2d::Zero();
    //TODO
	result(0) = point(0) / point(2);
	result(1) = point(1) / point(2);
    return result;
}
/**
 * \brief Assigns the values of the camera's extrinsic and intrinsic parameters.
 * \param[in] alpha The camera's current rotation angle.
 * \return a struct 'cameraParameters' which contains the camera parameters.
 */
CameraParameters ProjectiveGeometry::setCameraParameters(const double alpha)
{
    CameraParameters results;
    //TODO
	double principle_point_x;
	double principle_point_y;
	double camera_constant_c;
	double scale_difference_m;
	principle_point_x = 400.0;
	principle_point_y = 300.0;
	camera_constant_c = 550.0;
	scale_difference_m = 0.0025;
	Eigen::Vector3d X0;
	X0(0) = 0.4; //because in centimeters
	X0(1) = 0;
	X0(2) = 10;
	
	//Need to check how to get rx,ry,rz
	//Since roatating in xz vertically.
	double rx = 0;
	double ry = alpha;
	double rz = 0;
	results = { principle_point_x,principle_point_y, scale_difference_m,camera_constant_c,
				X0, rx,ry,rz};
    return results;
}
/**
 * \brief Computes the calibration matrix based on the camera's intrinsic parameters.
 * \param[in] camera parameters (CameraParameters struct).
 * \return Calibration matrix.
 */
Eigen::Matrix3d ProjectiveGeometry::calibrationMatrix(const CameraParameters& param)
{
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d ideal_calibration = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d sHc = Eigen::Matrix3d::Zero();
    //TODO
	ideal_calibration.row(0) << param.c, 0, 0;
	ideal_calibration.row(1) << 0, param.c, 0;
	ideal_calibration.row(2) << 0, 0, 1;
	sHc.row(0) << 1, 0, param.xH;
	sHc.row(1) << 0, 1+param.m, param.yH;
	sHc.row(2) << 0, 0, 1;
	result = sHc * ideal_calibration;
    return result;
}

/**
 * \brief Computes the projection matrix based on the camera's parameters and the pre-computed calibration matrix.
 * \param[in] Calibration matrix.
 * \param[in] Camera parameters (cameraParameters struct).
 * \return Projection matrix.
 */
Eigen::MatrixXd ProjectiveGeometry::projectionMatrix(const Eigen::Matrix3d& calibrationMatrix, const CameraParameters& param)
{
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3, 4);

	Eigen::Matrix3d rx,ry,rz,rf;
	rx.row(0) << 1, 0, 0;
	rx.row(1) << 0, cos(param.rotX), -sin(param.rotX);
	rx.row(2) << 0, sin(param.rotX), cos(param.rotX);

	ry.row(0) << cos(param.rotY), 0, sin(param.rotY);
	ry.row(1) << 0, 1, 0;
	ry.row(2) << -sin(param.rotY), 0, cos(param.rotY);

	rz.row(0) << cos(param.rotZ), -sin(param.rotZ), 0;
	rz.row(1) << sin(param.rotZ), cos(param.rotZ), 0;
	rz.row(2) << 0, 0, 1;

	rf = rz*ry*rx;

	Eigen::MatrixXd i_x = Eigen::MatrixXd::Zero(3, 4);
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

	i_x << identity, -param.X0;

	result = calibrationMatrix*rf*i_x;

    //TODO

    return result;
}
/**
 * \brief Applies the pre-computed projection matrix on a 3D point in Euclidean coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \param[in] Projection matrix.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::projectPoint(const Eigen::Vector3d& point, const Eigen::MatrixXd& projectionMatrix)
{
    Eigen::Vector2d result = Eigen::Vector2d::Zero();

	 return homogeneousToEuclidean(projectionMatrix * euclideanToHomogeneous(point));


    return result;
}

/**
 * \brief Reprojects an image pixel to a 3D point on a given horizontal plane in Euclidean coordinates.
 * \param[in] imagePoint The image point in pixels where a feature (e.g., the corner of a chess board) was detected.
 * \param[in] calibrationMatrix The calibration matrix calculated above.
 * \param[in] param The intrinsic and extrinsic camera parameters.
 * \param[in] tableHeight The height of the table in front of the robot. The reprojected point will lie on that table.
 * \return 3D point in Eucldiean coordinates.
 */
Eigen::Vector3d ProjectiveGeometry::reprojectTo3D(const Eigen::Vector2d& imagePoint, const Eigen::Matrix3d& calibrationMatrix,
		const CameraParameters& param, const double tableHeight) {
    Eigen::Vector3d coordinates3D = Eigen::Vector3d::Zero();

    // TODO

	 double u, v, w;
	 Eigen::Matrix3d A1 = Eigen::Matrix3d::Zero();
	 Eigen::Vector3d X = Eigen::Vector3d::Zero();
	 A1.row(0) << 1, 0,-1*param.X0(0) / tableHeight;
	 A1.row(1) << 0, 1, -1*param.X0(1) / tableHeight;
	 A1.row(2) << 0, 0, 1 - (param.X0(2)/tableHeight);
	// A1.row(0) << 1, 0, 0;
	// A1.row(1) << 0, 1, 0;
	// A1.row(2) << 0, 0, 1;
	 Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero();
	 rotation_matrix.row(0) << cos(param.rotY), 0, sin(param.rotY);
	 rotation_matrix.row(1) << 0, 1, 0;
	 rotation_matrix.row(2) << -1 * sin(param.rotY), 0, cos(param.rotY);
	 Eigen::Matrix3d inter_result = Eigen::Matrix3d::Zero();
	 inter_result = calibrationMatrix * rotation_matrix * A1;
	 Eigen::Vector3d imageP;
	 imageP << imagePoint(0), imagePoint(1), 1;
	 X = inter_result.inverse() * imageP;
	 double t;
	 t = X[2] / tableHeight;
	 coordinates3D[0] = X[0] / t;
	 coordinates3D[1] = X[1] / t;
	 coordinates3D[2] = tableHeight; //as X[2]/t will result in tableHeight

	
	return coordinates3D;
}


}  // namespace projective_geometry
