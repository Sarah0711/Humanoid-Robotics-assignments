#include <gtest/gtest.h>
#include <cmath>
#include <signed_distance_function/SignedDistanceFunction.h>

using namespace signed_distance_function;

TEST(TestSignedDistanceFunction, calculateDistance) {
	if (SignedDistanceFunction::calculateDistance(Eigen::Vector2d(2., 3.), Eigen::Vector2d(3., 4.)) == 0.0) {
		FAIL() << "The method did not return the Euclidean distance.";
	}
	ASSERT_DOUBLE_EQ(sqrt(2.0), SignedDistanceFunction::calculateDistance(Eigen::Vector2d(2., 3.), Eigen::Vector2d(3., 4.)));
	ASSERT_DOUBLE_EQ(4.0 * sqrt(2.0), SignedDistanceFunction::calculateDistance(Eigen::Vector2d(2., 3.), Eigen::Vector2d(-2., -1.)));
}

TEST(TestSignedDistanceFunction, truncateDistance) {
	if (SignedDistanceFunction::truncateDistance(0.5, 2.0) == 0.0 && SignedDistanceFunction::truncateDistance(-0.5, 2.0) == 0.0) {
		FAIL() << "The method did not return the truncated signed distance.";
	}
	ASSERT_DOUBLE_EQ(-2.0, SignedDistanceFunction::truncateDistance(-2.5, 2.0));
	ASSERT_DOUBLE_EQ(-1.5, SignedDistanceFunction::truncateDistance(-1.5, 2.0));
	ASSERT_DOUBLE_EQ(-0.5, SignedDistanceFunction::truncateDistance(-0.5, 2.0));
	ASSERT_DOUBLE_EQ( 0.0, SignedDistanceFunction::truncateDistance( 0.0, 2.0));
	ASSERT_DOUBLE_EQ( 0.5, SignedDistanceFunction::truncateDistance( 0.5, 2.0));
	ASSERT_DOUBLE_EQ( 1.5, SignedDistanceFunction::truncateDistance( 1.5, 2.0));
	ASSERT_DOUBLE_EQ( 2.0, SignedDistanceFunction::truncateDistance( 2.5, 2.0));

	ASSERT_DOUBLE_EQ(-1.5, SignedDistanceFunction::truncateDistance(-2.5, 1.5));
	ASSERT_DOUBLE_EQ(-1.5, SignedDistanceFunction::truncateDistance(-1.5, 1.5));
	ASSERT_DOUBLE_EQ(-0.5, SignedDistanceFunction::truncateDistance(-0.5, 1.5));
	ASSERT_DOUBLE_EQ( 0.0, SignedDistanceFunction::truncateDistance( 0.0, 1.5));
	ASSERT_DOUBLE_EQ( 0.5, SignedDistanceFunction::truncateDistance( 0.5, 1.5));
	ASSERT_DOUBLE_EQ( 1.5, SignedDistanceFunction::truncateDistance( 1.5, 1.5));
	ASSERT_DOUBLE_EQ( 1.5, SignedDistanceFunction::truncateDistance( 2.5, 1.5));
}

TEST(TestSignedDistanceFunction, calculateWeight) {
	if (SignedDistanceFunction::calculateWeight(0.0, 2.5, 0.5) == 0.0
			&& SignedDistanceFunction::calculateWeight(1.5, 2.5, 0.5) == 0.0
			&& SignedDistanceFunction::calculateWeight(2.0, 2.5, 0.5) == 0.0) {
		FAIL() << "The method did not return the weight.";
	}
	ASSERT_DOUBLE_EQ(1.0, SignedDistanceFunction::calculateWeight(0.0, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(1.0, SignedDistanceFunction::calculateWeight(0.5, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(0.75, SignedDistanceFunction::calculateWeight(1.0, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(0.5, SignedDistanceFunction::calculateWeight(1.5, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(0.25, SignedDistanceFunction::calculateWeight(2.0, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(0.0, SignedDistanceFunction::calculateWeight(2.5, 2.5, 0.5));
	ASSERT_DOUBLE_EQ(0.0, SignedDistanceFunction::calculateWeight(3.0, 2.5, 0.5));
}

TEST(TestSignedDistanceFunction, updateMap) {
	if (SignedDistanceFunction::updateMap(-2.0, 3.0, 5.0, 4.0) == 0.0) {
		FAIL() << "The method did not return the new signed distance.";
	}
	ASSERT_DOUBLE_EQ(2.0, SignedDistanceFunction::updateMap(-2.0, 3.0, 5.0, 4.0));
}

TEST(TestSignedDistanceFunction, updateWeight) {
	if (SignedDistanceFunction::updateWeight(2.0, 4.0) == 0.0) {
		FAIL() << "The method did not return the new weight.";
	}
	ASSERT_DOUBLE_EQ(6.0, SignedDistanceFunction::updateWeight(2.0, 4.0));
}

TEST(TestSignedDistanceFunction, integrateLaserScan_map) {
	{
    Eigen::MatrixXd map = Eigen::MatrixXd::Zero(20, 20);
    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(20, 20);
    Measurement measurement;
    const size_t rx = 3, y = 10;
    const size_t lx = 13;
    measurement.robotPosition << rx, y;
    const Eigen::Vector2d laserPoint(lx, y);

    measurement.laserPoints.push_back(laserPoint);
    SignedDistanceFunction::integrateLaserScan(map, weights, measurement);

    if (map.isZero(1e-5)) {
    	FAIL() << "The method does not update the map at all.";
    }
    if (!map.topRows(y).isZero(1e-5) || !map.bottomRows(map.rows() - y - 1).isZero(1e-5)) {
    	FAIL() << "The method updates cells that are not on the straight line between the robot and the laser."
    			<< "Make sure that the x axis corresponds to columns and the y axis corresponds to rows.";
    }
    if (map(y, rx) == 0.0 && fabs((double) map(y, rx - 1)) == 1.0 && fabs((double) map(y, rx + 1)) == 1.0) {
       FAIL() << "The signed distance should be measured from the laser endpoint, not from the robot.";
    }

    const Eigen::VectorXd before = map.block(y, 0, 1, rx).transpose();
    const Eigen::VectorXd between = map.block(y, rx + 1, 1, lx - rx - 2).transpose();
    const Eigen::VectorXd beyond = map.block(y, lx + 1, 1, map.cols() - (lx + 1)).transpose();

    if (!before.isZero(1e-5)) {
    	FAIL() << "The implementation should not update the cells behind the robot.";
    }
    if (between.isZero(1e-5)) {
        FAIL() << "The implementation should update the cells between the robot and the laser end point.";
    }
    if (fabs((double) between(0)) == 1.0 && fabs((double) between(1)) == 2.0) {
    	FAIL() << "The signed distance should be measured between the cell and the laser end point, not between the cell and the robot.";
    }

    if (beyond.isZero(1e-5)) {
    	FAIL() << "The implementation should update the cells beyond the laser endpoint.";
    }
    if (between.maxCoeff() > 0.0) {
        FAIL() << "The cells between the robot and the laser end point should get a negative signed distance";
    }
    if (beyond.minCoeff() < 0.0) {
        FAIL() << "The cells beyond the laser end point should get a positive signed distance.";
    }

    Eigen::MatrixXd mapExpected = Eigen::MatrixXd::Zero(20, 20);
    mapExpected.row(y)     << 0,  0,  0, -5, -5, -5, -5, -5, -5, -4, -3, -2, -1,  0,  1,    2,   3,    4,  0,  0;
    for (int row = 0; row < map.rows(); ++row) {
		for (int col = 0; col < map.cols(); ++col) {
			ASSERT_DOUBLE_EQ((double) mapExpected(row, col), (double) map(row, col));
		}
    }
	}

    {
    	// left
    	Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(3, 10);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd mapExpected2 = Eigen::MatrixXd::Zero(20, 20);
		mapExpected2.row(10) << 3,  2,  1,  0, -1, -2, -3, -4, -5, -5, -5, -5, -5, -5,  0,  0,  0,  0,  0,  0;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) mapExpected2(row, col), (double) map2(row, col)) << "The method fails when the robot is looking in negative x direction.";
			}
		}
    }

    {
    	// down
    	Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(13, 17);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd mapExpected2 = Eigen::MatrixXd::Zero(20, 20);
		mapExpected2.col(13) << 0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -5, -5, -5, -4, -3, -2, -1,  0,  1,  2;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) mapExpected2(row, col), (double) map2(row, col)) << "The method fails when the robot is looking in positive y direction.";
			}
		}
    }

    {
    	// up
    	Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(13, 3);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd mapExpected2 = Eigen::MatrixXd::Zero(20, 20);
		mapExpected2.col(13) << 3,  2,  1,  0, -1, -2, -3, -4, -5, -5, -5,  0,  0,  0,  0,  0,  0,  0,  0,  0;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) mapExpected2(row, col), (double) map2(row, col))  << "The method fails when the robot is looking in negative y direction.";
			}
		}
    }
}

TEST(TestSignedDistanceFunction, integrateLaserScan_weights) {
	{
	    Eigen::MatrixXd map = Eigen::MatrixXd::Zero(20, 20);
	    Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(20, 20);
	    Measurement measurement;
	    const size_t rx = 3, y = 10;
	    const size_t lx = 13;
	    measurement.robotPosition << rx, y;
	    const Eigen::Vector2d laserPoint(lx, y);

	    measurement.laserPoints.push_back(laserPoint);
	    SignedDistanceFunction::integrateLaserScan(map, weights, measurement);

	    const Eigen::VectorXd weightsBefore = weights.block(y, 0, 1, rx).transpose();
	    const Eigen::VectorXd weightsBetween = weights.block(y, rx + 1, 1, lx - rx - 2).transpose();
		const Eigen::VectorXd weightsBeyond = weights.block(y, lx + 1, 1, weights.cols() - (lx + 1)).transpose();

		if (weights.isZero(1e-5)) {
			FAIL() << "The method does not update the weights at all.";
		}

	    if (!weights.topRows(y).isZero(1e-5) || !weights.bottomRows(map.rows() - y - 1).isZero(1e-5)) {
	    	FAIL() << "The method updates cells that are not on the straight line between the robot and the laser."
	    			<< "Make sure that the x axis corresponds to columns and the y axis corresponds to rows.";
	    }

		if (weights.minCoeff() < 0.0) {
			FAIL() << "Some weights are negative, which is not allowed.";
		}
		if (!weightsBefore.isZero(1e-5)) {
			FAIL() << "The method should not update the weights behind the robot.";
		}
		if (weightsBetween.isZero()) {
			FAIL() << "The method should update the weights between the robot and the laser end point.";
		}
		if (weightsBeyond.isZero()) {
			FAIL() << "The method should update the weights between the robot and the laser end point.";
		}

	    if (weights(y, lx - 1) != 1.0) {
			FAIL() << "The implementation does not update the weights correctly between the robot and the laser end point.";
		}
		if (weights(y, lx + 2) != 0.75) {
	        FAIL() << "The implementation does not update the weights correctly beyond the laser end point.";
	    }

	    Eigen::MatrixXd weightsExpected = Eigen::MatrixXd::Zero(20, 20);
	    weightsExpected.row(y) << 0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 0.75, 0.5, 0.25,  0,  0;
	    for (int row = 0; row < map.rows(); ++row) {
			for (int col = 0; col < map.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) weightsExpected(row, col), (double) weights(row, col));
			}
	    }
	}

	{
		// left
		Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(5, 10);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd weightsExpected2 = Eigen::MatrixXd::Zero(20, 20);
		weightsExpected2.row(10) << 0, 0.25,  0.5, 0.75,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    0,    0,    0,    0,    0,    0;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) weightsExpected2(row, col), (double) weights2(row, col)) << "The method fails when the robot is looking in negative x direction.";
			}
		}
	}

	{
		// down
		Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(13, 15);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd weightsExpected2 = Eigen::MatrixXd::Zero(20, 20);
		weightsExpected2.col(13) << 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    1, 0.75,  0.5, 0.25;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) weightsExpected2(row, col), (double) weights2(row, col)) << "The method fails when the robot is looking in positive y direction.";
			}
		}
	}

	{
		// up
		Measurement measurement2;
		measurement2.robotPosition << 13, 10;
		const Eigen::Vector2d laserPoint2(13, 5);
		measurement2.laserPoints.push_back(laserPoint2);
		Eigen::MatrixXd map2 = Eigen::MatrixXd::Zero(20, 20);
		Eigen::MatrixXd weights2 = Eigen::MatrixXd::Zero(20, 20);
		SignedDistanceFunction::integrateLaserScan(map2, weights2, measurement2);
		Eigen::MatrixXd weightsExpected2 = Eigen::MatrixXd::Zero(20, 20);
		weightsExpected2.col(13) << 0, 0.25,  0.5, 0.75,    1,    1,    1,    1,    1,    1,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0;
		for (int row = 0; row < map2.rows(); ++row) {
			for (int col = 0; col < map2.cols(); ++col) {
				ASSERT_DOUBLE_EQ((double) weightsExpected2(row, col), (double) weights2(row, col))  << "The method fails when the robot is looking in negative y direction.";
			}
		}
	}
}


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

