#include <odometry_calibration/OdometryCalibration.h>

#include <cmath>
#include <iostream>


namespace odometry_calibration {

/**
 * \brief Computes the odometry error function.
 * \param[in] groundTruth The ground truth odometry measured by an external sensor (called u* in the slides).
 * \param[in] observation The odometry measurement observed by the robot itself, e.g., using wheel sensors or joint angles (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix of the current iteration (called X in the slides).
 * \return The error function vector (called e in the slides).
 */
Eigen::Vector3d OdometryCalibration::errorFunction(const Odometry& groundTruth, const Odometry& observation, const Eigen::Matrix3d& calibrationMatrix) {
	Eigen::Vector3d error;

	Eigen::Vector3d observationVector;
	observationVector<<observation.ux, observation.uy, observation.utheta;

	Eigen::Vector3d groundTruthVector;
	groundTruthVector<<groundTruth.ux, groundTruth.uy, groundTruth.utheta;
	error = groundTruthVector - (calibrationMatrix * observationVector);
	return error;
}

/**
 * \brief Computes the Jacobian (matrix derivative) of the error function for a given odometry measurement.
 * \param[in] measurement The odometry measured by the robot (called u in the slides).
 * \return The Jacobian matrix of the error function.
 */
Eigen::Matrix3Xd OdometryCalibration::jacobian(const Odometry& observation) {
	Eigen::Matrix3Xd jacobian(3, 9);
	jacobian.row(0) << double(observation.ux), double(observation.uy), double(observation.utheta), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	jacobian.row(1) << 0.0, 0.0, 0.0, double(observation.ux), double(observation.uy), double(observation.utheta), 0.0, 0.0, 0.0;
	jacobian.row(2) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, double(observation.ux), double(observation.uy), double(observation.utheta);
	jacobian = jacobian * -1;
	//TODO: Calculate the 3x9 Jacobian matrix of the error function for the given observation.
	return jacobian;
}

/**
 * \brief Calibrates the odometry of a robot.
 * \param[in] measurementData A vector containing ground truth and observed odometry measurements.
 * ŗeturn The calibration matrix that can be used to correct the odometry.
 */
Eigen::Matrix3d OdometryCalibration::calibrateOdometry(const std::vector<MeasurementData>& measurements) {
	Eigen::Matrix3d calibrationMatrix = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d weight_matrix = Eigen::Matrix3d::Identity();
	Eigen::MatrixXd H =  Eigen::MatrixXd::Zero(9,9);
	Eigen::MatrixXd b_trans = Eigen::MatrixXd::Zero(1,9);
	
	/** TODO: Calculate the calibration matrix. The steps to do are:
	 * - Start with an arbitrary initial calibration matrix
	 * - Iterate through the calibration data
	 * - Compute the error function and Jacobian for each data set
	 * - Accumulate the linear system components H and b
	 * - Solve the linear system
	 * - Update the calibration matrix
	 */


	Eigen::Vector3d error_calc;
	Eigen::MatrixXd jacobian_calc(3, 9);

	for(int i=0; i<measurements.size();i++){
		MeasurementData measurement = measurements[i];
		Odometry groundTruth = measurement.groundTruth;
		Odometry uncalibrated = measurement.uncalibrated;

		error_calc = errorFunction(groundTruth,uncalibrated,calibrationMatrix);
		jacobian_calc = jacobian(uncalibrated);
		b_trans = b_trans + error_calc.transpose() * weight_matrix * jacobian_calc;
		H = H + jacobian_calc.transpose() * weight_matrix * jacobian_calc;

		// Eigen::Map<Matrix3d> delta_mat(delta_x.data());
	 	// calibrationMatrix = calibrationMatrix + delta_mat;

	}
		Eigen::VectorXd delta_x;
		delta_x = -1 * H.inverse()  * b_trans.transpose();
		// std::cout<<delta_x<<std::endl<<std::endl;
		Eigen::Map<Eigen::MatrixXd> delta_mat(delta_x.data(),3,3);

		calibrationMatrix = calibrationMatrix + delta_mat;



	return calibrationMatrix;
}

/**
 * \brief Applies the calibration matrix to an odometry measurement in order to get a corrected estimate.
 * \param[in] uncalibratedOdometry An uncalibrated odometry measurement (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix computed by calibrateOdometry in a previous step (called X in the slides).
 * \return The corrected odometry estimate.
 */
Odometry OdometryCalibration::applyOdometryCorrection(const Odometry& uncalibratedOdometry, const Eigen::Matrix3d& calibrationMatrix) {
	Odometry calibratedOdometry;
	Eigen::Vector3d uncalibratedOdometryVector;
	uncalibratedOdometryVector<<uncalibratedOdometry.ux, uncalibratedOdometry.uy, uncalibratedOdometry.utheta;
	Eigen::Vector3d calibratedOdometryVector = calibrationMatrix * uncalibratedOdometryVector;
	calibratedOdometry.ux = double(calibratedOdometryVector(0));
	calibratedOdometry.uy = double(calibratedOdometryVector(1));
	calibratedOdometry.utheta = double(calibratedOdometryVector(2));
	return calibratedOdometry;
}

/**
 * \brief Converts an odometry reading into an affine 2D transformation.
 * \param[in] odometry The odometry reading.
 * \returns The corresponding affine transformation as a 3x3 matrix.
 */
Eigen::Matrix3d OdometryCalibration::odometryToAffineTransformation(const Odometry& odometry) {
	Eigen::Matrix3d transformation;
	//TODO: Convert the odometry measurement to an affine transformation matrix.

	transformation.row(0)<<cos(odometry.utheta), -sin(odometry.utheta), odometry.ux;
	transformation.row(1)<<sin(odometry.utheta), cos(odometry.utheta), odometry.uy;
	transformation.row(2)<<0, 0, 1;

	return transformation;
}

/**
 * \brief Converts an affine 2D transformation matrix into a 2D robot pose (x, y, and theta).
 * \param[in] transformation An affine transformation as a 3x3 matrix.
 * \returns The corresponding 2D pose (x, y, and theta).
 */
Pose2D OdometryCalibration::affineTransformationToPose(const Eigen::Matrix3d& transformation) {
	Pose2D pose;
	/* TODO: replace the following lines by the x and y position and the rotation of the robot.
	 * Hint: x and y can be directly read from the matrix. To get the rotation, use the acos/asin
	 * functions on the rotation matrix and take extra care of the sign, or (better) use the
	 * atan2 function.
	 */
	pose.x = transformation(0,2);
	pose.y = transformation(1,2);
	pose.theta = acos(transformation(0,0));
	return pose;
}

/**
 * \brief Calculate the robot's trajectory in Cartesian coordinates given a list of calibrated odometry measurements.
 * \param[in] calibratedOdometry Odometry measurements that have already been corrected using the applyOdometryCorrection method.
 * \returns A vector of 2D poses in the global coordinate system.
 */
std::vector<Pose2D> OdometryCalibration::calculateTrajectory(const std::vector<Odometry>& calibratedOdometry) {
	std::vector<Pose2D> trajectory;
	/* TODO: Compute the trajectory of the robot.
	 * - Start at the position x = 0, y = 0, theta = 0. (Do not add this point to the trajectory).
	 * - Iterate through the odometry measurements.
	 * - Convert each odometry measurement to an affine transformation using the
	 *   odometryToAffineTransformation method from above.
	 * - Chain the affine transformation to get the next pose.
	 * - Convert the affine transformation back to a 2D pose using the
	 *   affineTransformationToPose method from above.
	 * - Store the pose in the trajectory vector.
	 */
	Eigen::Matrix3d chained_transformation = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d affine_transformation;
	Pose2D returned_pose;
	for (int i = 0; i < calibratedOdometry.size(); i++) {
		Odometry odd = calibratedOdometry[i];
		affine_transformation = odometryToAffineTransformation(odd);
		chained_transformation = chained_transformation * affine_transformation;
		returned_pose=affineTransformationToPose(chained_transformation);
		trajectory.push_back(returned_pose);
	}
	return trajectory;
}




} /* namespace odometry_calibration */
