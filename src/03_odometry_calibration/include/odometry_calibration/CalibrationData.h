#ifndef CALIBRATIONDATA_H_
#define CALIBRATIONDATA_H_

namespace odometry_calibration {

struct Pose2D {
	double x;
	double y;
	double theta;

	Pose2D() : x(0.0), y(0.0), theta(0.0) {}
};

struct Odometry {
	double ux;
	double uy;
	double utheta;
	
	Odometry() : ux(0.0), uy(0.0), utheta(0.0) {}
};

struct MeasurementData {
	Odometry groundTruth;
	Odometry uncalibrated;
};

}  // namespace odometry_calibration

#endif /* CALIBRATIONDATA_H_ */
