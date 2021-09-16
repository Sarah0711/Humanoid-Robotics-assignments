#include <windows-helpers.h>
#include <odometry_calibration/FileIO.h>
#include <odometry_calibration/OdometryCalibration.h>
#include <odometry_calibration/CalibrationData.h>
#include <iostream>
#include <stdexcept>

using namespace odometry_calibration;

int main(int /* argc */, char ** /*argv*/) {
	std::string packagePath = "../";

	FileIO fileIO;
	try {
		fileIO.loadFromFile(packagePath + "/data/calib.txt");
	} catch(const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		wait();
		return 2;
	}

	std::cout << "Calibrating..." << std::endl;
	Eigen::Matrix3d calibrationMatrix = OdometryCalibration::calibrateOdometry(fileIO.measurements);
	std::cout << "Calibration finished. The calibration matrix is: " << std::endl;
	std::cout << calibrationMatrix << std::endl;
	std::vector<Odometry> calibratedOdometry;
	calibratedOdometry.reserve(fileIO.uncalibrated.size());
	for (std::vector<Odometry>::const_iterator it = fileIO.uncalibrated.begin(); it != fileIO.uncalibrated.end(); ++it) {
		calibratedOdometry.push_back(OdometryCalibration::applyOdometryCorrection(*it, calibrationMatrix));
	}
	try {
		fileIO.writeToFile(packagePath + "/data/trajectory.txt", calibratedOdometry);
	} catch(const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		wait();
		return 3;
	}

	wait();
	return 0;

}
