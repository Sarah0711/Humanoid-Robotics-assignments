#include <windows-helpers.h>
#include <iostream>
#include <inverse_kinematics/InverseKinematics.h>
#include <angles/angles.h>
#include <fstream>

using namespace inverse_kinematics;

void run2links() {
	const double alpha = 0.1;
	const double maxError = 0.01;

	const double a0 = 3.0;
	const double a1 = 2.0;
	const double h = 0.5;
	const InverseKinematics_2Links ik2(a0, a1, h, alpha);
	const EndeffectorPose goal2(Eigen::Vector2d(3.0, 3.0));

	const JointAngles q = ik2.computeIK(goal2, maxError);
	std::cout << "Kinematic chain with 2 links: " << std::endl;
	std::cout << "Goal: (" << goal2.transpose(); std::cout << ")" << std::endl;
	std::cout << "Computed joint angles: (" << q.transpose(); std::cout << ")" << std::endl;
	std::cout << "Resulting pose: (" << ik2.forwardKinematic(q).transpose() << ")"; std::cout << ")" << std::endl;
	std::cout << std::endl;
}

void run3links() {
	const double alpha = 0.1;
	const double maxError = 0.01;
	const double maxAngularError = angles::from_degrees(1.0);

	const double a0 = 1.6;
	const double a1 = 1.2;
	const double a2 = 0.7;
	const double h = 0.1;

	const InverseKinematics_3Links ik3(a0, a1, a2, h, alpha);
	const EndeffectorPose goal3(Eigen::Vector3d(2.5, 1.5, 0.0));
	const JointAngles q = ik3.computeIK(goal3, maxError, maxAngularError);
	std::cout << "Kinematic chain with 3 links: " << std::endl;
	std::cout << "Goal: (" << goal3.transpose(); std::cout << ")" << std::endl;
	std::cout << "Computed joint angles: (" << q.transpose(); std::cout << ")" << std::endl;
	std::cout << "Resulting pose: (" << ik3.forwardKinematic(q).transpose() << ")"; std::cout << ")" << std::endl;
}

void runGlass() {
	const std::string packagePath = PROJECT_SOURCE_DIR;
	const std::string filename = packagePath + "/data/glass.txt";
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open " << filename << " for writing log file." << std::endl;
		return;
	}

	const double alpha = 0.1;
	const double maxError = 0.01;
	const double maxAngularError = angles::from_degrees(1.0);

	const double a0 = 1.6;
	const double a1 = 1.2;
	const double a2 = 0.7;
	const double h = 0.1;

	const InverseKinematics_3Links ik3(a0, a1, a2, h, alpha);

	const double rx1 = 0.1, ry1 = 0.2;
	const double rx2 = 0.1, ry2 = 0.2;
	const double tx = 2.5, ty = 0.8;
	const double k = 0.1;
	for (double angle = 0; angle <= 2.0 * M_PI; angle += 0.01 * M_PI) {
		double x, y;
		if (angle <= 0.5 * M_PI) {
			x = tx + rx1 * cos(angle);
			y = ty + ry1 * sin(angle);
		} else {
			x = tx + (rx2 + k*(angle - 0.5 * M_PI)) * cos(2.0 * M_PI - angle);
			y = ty + ry1 + ry2 + ry2 * sin(2.0 * M_PI - angle);
		}

		const EndeffectorPose goal3(Eigen::Vector3d(x, y, 0.0));
		const JointAngles q = ik3.computeIK(goal3, maxError, maxAngularError);
		for (size_t i = 0; i < 3; ++i) {
			ofs << angles::normalize_angle(static_cast<double>(q(i))) << " ";
		}
		ofs << ik3.forwardKinematic(q).transpose();
		ofs << std::endl;
	}
	ofs.close();
	std::cout << std::endl << "Wrote joint angles for carrying glass to " << filename.c_str() << std::endl;
}

int main(int /* argc */, char ** /*argv*/)
{
	run2links();
	run3links();
	runGlass();

    wait();
	return 0;
}
