#include <windows-helpers.h>
#include <signed_distance_function/SignedDistanceFunction.h>
#include <signed_distance_function/FileIO.h>
#include <iostream>

using namespace signed_distance_function;

int main(int /* argc */, char ** /*argv*/) {
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath + "/data/data.txt");
	SignedDistanceFunction sdf;
	Eigen::MatrixXd map = Eigen::MatrixXd::Zero(fileIO.sizeX, fileIO.sizeY);
	Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(fileIO.sizeX, fileIO.sizeY);

	for(FileIO::MeasurementsVector::const_iterator it = fileIO.measurements.begin(); it != fileIO.measurements.end(); ++it) {
		sdf.integrateLaserScan(map, weights, *it);
	}
	fileIO.writeMap(map, packagePath + "/data/result.txt");

    wait();
	return 0;
}
