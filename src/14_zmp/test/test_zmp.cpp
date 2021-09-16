#include <gtest/gtest.h>
#include <zmp/ZMP.h>

using namespace zmp;

TEST(ZMP, giForce) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << 3.0, 4.0, 5.0;
	const Force F = zmp.giForce();
	if (F.isApprox(Force::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}
	if (F.isApprox(Force(0.0, 0.0, 19.62), 1e-2)) {
		FAIL() << "The method does not take into account the external force.";
	}
	if (F.isApprox(zmp.F_E)) {
		FAIL() << "The method does not take into account the gravitational force.";
	}
	if (F.isApprox(Force(3.0, 4.0, 14.81), 1e-2)) {
		FAIL() << "The method does not take into account the mass of the robot.";
	}
	if (F.isApprox(Force(3.0, 4.0, 24.62), 1e-2)) {
		FAIL() << "The method uses the wrong direction for the gravitational force. Note that g = (0, 0, -9.81).";
	}

	if (F.isApprox(Force(3.0, 4.0, -14.62), 1e-2)) {
		SUCCEED();
	} else {
		FAIL() << "The method returned an incorrect result.";
	}
}

TEST(ZMP, contactForce) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << 3.0, 4.0, 5.0;
	const Force F = zmp.giForce();
	const Force CF = zmp.contactForce();
	if (CF.isApprox(Force::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}
	if (!(F+CF).isApprox(Force::Zero())) {
		FAIL() << "The method returns a result that violates Newton's first law for the robot standing still.";
	}
}

TEST(ZMP, giMoment) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << 3.0, 4.0, 5.0;
	zmp.C << 0, 0, 0.5;
	const Point3d X(-0.1, -0.05, 0.1);
	const AngularMoment M = zmp.giMoment(X);
	if (M.isApprox(AngularMoment::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}

	if (M.isApprox(AngularMoment(-0.981, 1.962, 0.000), 1e-2)) {
		FAIL() << "The method does not take into account the torque induced by the external force.";
	}
	if (M.isApprox(AngularMoment(0.65, -0.8, 0.25), 1e-2)) {
		FAIL() << "The method does not take into account the torque induced by the gravitational force.";
	}
	if (M.isApprox(AngularMoment(-2.331, 2.662, 0.25), 1e-2)) {
		FAIL() << "The method applies the external force in the center of mass, but it is applied in point E on the robot's chest.";
	}
	if (M.isApprox(AngularMoment(0.331, -1.162, -0.25), 1e-2)) {
		FAIL() << "The sign of the result is incorrect. The lever arm vector goes from the point of reference X to the point where the force is applied.";
	}
	if (M.isApprox(AngularMoment(0.1595, 0.181, 0.25), 1e-2)) {
		FAIL() << "The method does not take into account the mass of the robot.";
	}
	if (M.isApprox(AngularMoment(-0.331, 1.162, 0.25), 1e-2)) {
		SUCCEED();
	} else {
		FAIL() << "The method returns an incorrect result.";
	}
}

TEST(ZMP, zeroMomentPoint) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << -3.0, 4.0, -0.5;
	zmp.E << 0.2, 0.2, 0.8;
	zmp.C << 0.1, 0.1, 0.5;
	zmp.O << -0.1, -0.05, 0.0;
	const Point3d X(-0.1, -0.05, 0.1);
	const Point3d Z = zmp.zeroMomentPoint(zmp.O);
	if (Z.isApprox(Point3d::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}
	if (Z.isApprox(Point3d(0.0832008, 0.311531, 0), 1e-2)) {
		FAIL() << "The method returns (Z-O) instead of Z.";
	}
	if (Z.isApprox(Point3d(0.183201, 0.361531, 0), 1e-2)) {
		FAIL() << "The method returns (Z+O) instead of Z.";
	}
	if (Z.isApprox(Point3d(0.211531, -0.133201, -0.0969185), 1e-2)) {
		FAIL() << "The method does not project M^gi to the contact plane.";
	}

	if (Z.isApprox(Point3d(-0.183201, -0.361531, 0), 1e-2)) {
		FAIL() << "The method used M x n instead of n x M in the numerator. The cross product is not commutative!";
	}
	if (Z.isApprox(AngularMoment(0.0167992, -0.261531, 0), 1e-2)) {
		FAIL() << "The sign of the result is incorrect.";
	}

	if (Z.isApprox(Point3d(-0.0167992, 0.261531, 0), 1e-2)) {
		SUCCEED();
	} else {
		FAIL() << "The method returns an incorrect result";
	}
}

TEST(ZMP, contactMoment) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << -3.0, 4.0, -0.5;
	zmp.E << 0.2, 0.2, 0.8;
	zmp.C << 0.1, 0.1, 0.5;
	zmp.O << -0.1, -0.05, 0.0;
	const Point3d X(-0.1, -0.05, 0.1);
	const Point3d CoP(-0.0167992, 0.1, 0);
	const AngularMoment M = zmp.contactMoment(CoP, X);
	if (M.isApprox(Point3d::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}
	if (M.isApprox(AngularMoment(4.618, -2.824, -1.25), 1e-2)) {
		FAIL() << "The method applied the contact force in the center of mass instead of the center of pressure.";
	}
	if (M.isApprox(AngularMoment(-2.618, 1.974, 0.782803), 1e-2)) {
		FAIL() << "The sign of the result is incorrect.";
	}
	if (fabs((double) M[2]) < 1e-3) {
		FAIL() << "The method does not take into account the moments induced by the friction.";
	}
	if (M.isApprox(AngularMoment(2.618, -1.974, -0.782803), 1e-2)) {
		SUCCEED();
	} else {
		FAIL() << "The method returns an incorrect result";
	}
}

TEST(ZMP, resultingMomentAtCoM) {
	ZMP zmp;
	zmp.m = 2.0;
	zmp.F_E << -3.0, 4.0, -0.5;
	zmp.E << 0.2, 0.2, 0.8;
	zmp.C << 0.1, 0.1, 0.5;
	zmp.O << -0.1, -0.05, 0.0;
	const Point3d X(-0.1, -0.05, 0.1);
	const Point3d CoP(-0.0167992, 0.1, 0);
	const AngularMoment M = zmp.resultingMomentAtCoM(CoP);
	if (M.isApprox(Point3d::Zero())) {
		FAIL() << "The method returns the zero vector.";
	}
	if (M.isApprox(AngularMoment(3.25, 0, -1.1672), 1e-2)) {
		FAIL() << "The sign of the result is incorrect.";
	}
	if (M.isApprox(AngularMoment(-3.25, 0, 1.1672), 1e-2)) {
		SUCCEED();
	} else {
		std::cout << M.transpose() << std::endl;
		FAIL() << "The method returns an incorrect result";
	}
}

int main(int argc, char *argv[]) {
    srand((unsigned int) time(NULL));
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
