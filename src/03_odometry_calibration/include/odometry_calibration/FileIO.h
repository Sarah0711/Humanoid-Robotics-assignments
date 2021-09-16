#ifndef FILEIO_H_
#define FILEIO_H_

#include <odometry_calibration/CalibrationData.h>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace odometry_calibration {

class FileIO {
public:
	FileIO() {};
	virtual ~FileIO() {};

	const std::vector<MeasurementData>& loadFromFile(const std::string& filename);
	void writeToFile(const std::string& filename, const std::vector<Odometry>& calibrated);

	std::vector<MeasurementData> measurements;
	std::vector<Odometry> uncalibrated;
};

} /* namespace odometry_calibration */

#endif /* FILEIO_H_ */
