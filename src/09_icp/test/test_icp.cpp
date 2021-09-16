#include <gtest/gtest.h>
#include <icp/ICP.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <math.h>
#include <vector>

using namespace icp;

TEST(ICP, distance) {
	const double epsilon = 0.0001;

	const double distance = ICP::distance(Eigen::Vector2d(-10, 10), Eigen::Vector2d(5, 7));
	ASSERT_GE(distance, 0.0) << "The method returned a negative distance.";

	ASSERT_NEAR( 15.2971, ICP::distance(Eigen::Vector2d(-10., 10.), Eigen::Vector2d(5., 7.)), epsilon);
	ASSERT_NEAR( 6.0902, ICP::distance(Eigen::Vector2d(-10., -3.3), Eigen::Vector2d(-7., 2.)), epsilon);
	ASSERT_NEAR( 2.0, ICP::distance(Eigen::Vector2d(0., 2.), Eigen::Vector2d(0., 0.)), epsilon);
	ASSERT_NEAR( 0.0, ICP::distance(Eigen::Vector2d(3., 5.), Eigen::Vector2d(3., 5.)), epsilon);
}

TEST(ICP, closestPointOnLine) {
	const double epsilon = 0.01;
	Eigen::Vector2d p0,p1,p2;

	p0<< 8,2 ;  p1<<-10,10;  p2<<5,7;
	if (ICP::closestPointOnLine(p0,p1,p2).isZero()) {
		FAIL() << "The method did not return the closest point.";
	}
	ASSERT_NEAR( 8.85, (double) ICP::closestPointOnLine(p0,p1,p2)(0), epsilon);
	ASSERT_NEAR( 6.23, (double) ICP::closestPointOnLine(p0,p1,p2)(1), epsilon);

	p0<< -6,3 ;  p1<<-10,-3.3;  p2<<-7,2;
	ASSERT_NEAR(-6.33, (double) ICP::closestPointOnLine(p0,p1,p2)(0), epsilon);
	ASSERT_NEAR( 3.19, (double) ICP::closestPointOnLine(p0,p1,p2)(1), epsilon);

	p0<< 4,-7 ;  p1<<0,2;  p2<<0,0;
	if (isnan((double) ICP::closestPointOnLine(p0,p1,p2)(0))) {
		FAIL() << "The implementation does not handle the special case of vertical lines (pL1.x == pL2.x) correctly.";
	}
	ASSERT_NEAR( 0, (double) ICP::closestPointOnLine(p0,p1,p2)(0), epsilon);
	ASSERT_NEAR(-7, (double) ICP::closestPointOnLine(p0,p1,p2)(1), epsilon);

	p0<< 1,1 ;  p1<<2,0;  p2<<0,0;
	if (isnan((double) ICP::closestPointOnLine(p0,p1,p2)(0))) {
		FAIL() << "The implementation does not handle the special case of horizontal lines (pL1.y == pL2.y) correctly.";
	}
	ASSERT_NEAR( 1, (double) ICP::closestPointOnLine(p0,p1,p2)(0), epsilon);
	ASSERT_NEAR( 0, (double) ICP::closestPointOnLine(p0,p1,p2)(1), epsilon);
}

TEST(ICP, minDistance)
{
	const double v1[12] = {12., -7., 5., 0.7, -8., -100., -0.1, 8., -100., 5., 0., 0.};
	std::vector<double> v(v1, v1 + 12);

	ASSERT_DOUBLE_EQ( -100.0, ICP::minDistance(v));

	std::vector<double> v2;
	v2.push_back(1.0);
	v2.push_back(2.0);
	v2.push_back(3.0);
	v2.push_back(4.0);
	if (ICP::minDistance(v2) == 2.0) {
		FAIL() << "The implementation fails if the first element is the minimum.";
	}

	std::vector<double> v3;
	v3.push_back(4.0);
	v3.push_back(3.0);
	v3.push_back(2.0);
	v3.push_back(1.0);
	if (ICP::minDistance(v3) == 2.0) {
		FAIL() << "The implementation fails if the last element is the minimum.";
	}
}

TEST(ICP, euclideanCorrespondences) 
{
	StdVectorOfVector2d Q1, P1;
	Q1.push_back(Eigen::Vector2d(1., 1.));
	Q1.push_back(Eigen::Vector2d(3., 3.));
	P1.push_back(Eigen::Vector2d(1.5, 1.));

	const StdVectorOfVector2d C1 = ICP::euclideanCorrespondences(Q1, P1);
	if (C1.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(Q1.size(), ICP::euclideanCorrespondences(Q1, P1).size());
	for (size_t i = 0; i < C1.size(); ++i) {
		if (C1[i].isApprox(Q1[0]) || C1[i].isApprox(Q1[1])) {
			FAIL() << "The method returned an element from Q, but it should only return elements from P.";
		}
		if (!C1[i].isApprox(P1[0])) {
			FAIL() << "The method returned an element that is neither in Q nor in P.";
		}
	}

	{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d P;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());

	const StdVectorOfVector2d C = ICP::euclideanCorrespondences(Q,P);

	if (C.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(Q.size(), C.size());
	ASSERT_DOUBLE_EQ( -0.5, (double) C[0](0) );
	ASSERT_DOUBLE_EQ(   3, 	(double) C[0](1) );
	ASSERT_DOUBLE_EQ( -0.5, (double) C[1](0) );
	ASSERT_DOUBLE_EQ(   3, 	(double) C[1](1) );
	ASSERT_DOUBLE_EQ( 0.4,  (double) C[2](0) );
	ASSERT_DOUBLE_EQ( 4.1,  (double) C[2](1) );
	ASSERT_DOUBLE_EQ( 1.3,  (double) C[3](0) );
	ASSERT_DOUBLE_EQ( 5.2,  (double) C[3](1) );
	ASSERT_DOUBLE_EQ( 4, 	(double) C[4](0) );
	ASSERT_DOUBLE_EQ( 4.5,  (double) C[4](1) );
	ASSERT_DOUBLE_EQ( 4.9,  (double) C[5](0) );
	ASSERT_DOUBLE_EQ( 3.6,  (double) C[5](1) );
	ASSERT_DOUBLE_EQ( 4.9,  (double) C[6](0) );
	ASSERT_DOUBLE_EQ( 3.6,  (double) C[6](1) );
	}

	{
		StdVectorOfVector2d Q2;
		StdVectorOfVector2d P2;
		Eigen::MatrixXd q2(7,2);
		q2<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
		Eigen::MatrixXd p2(7,2);
		p2<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 3,4, 4.9,3.6;
		for(int i=0;i<q2.rows();i++)
			Q2.push_back((q2.block<1,2>(i,0)).transpose());
		for(int i=0;i<p2.rows();i++)
			P2.push_back((p2.block<1,2>(i,0)).transpose());

		const StdVectorOfVector2d C2 = ICP::euclideanCorrespondences(Q2,P2);

		if (C2.empty()) {
			FAIL() << "The method returns an empty vector.";
		}
		ASSERT_EQ(Q2.size(), C2.size());
		ASSERT_DOUBLE_EQ( -0.5, (double) C2[0](0) );
		ASSERT_DOUBLE_EQ(   3, 	(double) C2[0](1) );
		ASSERT_DOUBLE_EQ( -0.5, (double) C2[1](0) );
		ASSERT_DOUBLE_EQ(   3, 	(double) C2[1](1) );
		ASSERT_DOUBLE_EQ( 0.4,  (double) C2[2](0) );
		ASSERT_DOUBLE_EQ( 4.1,  (double) C2[2](1) );
		ASSERT_DOUBLE_EQ( 1.3,  (double) C2[3](0) );
		ASSERT_DOUBLE_EQ( 5.2,  (double) C2[3](1) );
		ASSERT_DOUBLE_EQ( 4.9,  (double) C2[5](0) );
		ASSERT_DOUBLE_EQ( 3.6,  (double) C2[5](1) );
		ASSERT_DOUBLE_EQ( 4.9,  (double) C2[6](0) );
		ASSERT_DOUBLE_EQ( 3.6,  (double) C2[6](1) );

		ASSERT_DOUBLE_EQ( 3, 	(double) C2[4](0) ) << "The method does not work when P and Q contain the same point.";
		ASSERT_DOUBLE_EQ( 4,    (double) C2[4](1) ) << "The method does not work when P and Q contain the same point.";
}

}

TEST(ICP, closestPointToLineCorrespondences) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d P;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());

	const StdVectorOfVector2d C = ICP::closestPointToLineCorrespondences(Q,P);

	if (C.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
    ASSERT_EQ(Q.size(), C.size());
	ASSERT_NEAR(-1.19, (double) C[0](0), 0.01);
	ASSERT_NEAR( 2.16, (double) C[0](1), 0.01);
	ASSERT_NEAR(-0.30, (double) C[1](0), 0.10);
	ASSERT_NEAR( 3.25, (double) C[1](1), 0.01);
	ASSERT_NEAR( 0.60, (double) C[2](0), 0.10);
	ASSERT_NEAR( 4.33, (double) C[2](1), 0.01);
	ASSERT_NEAR( 1.48, (double) C[3](0), 0.01);
	ASSERT_NEAR( 5.42, (double) C[3](1), 0.01);
	ASSERT_DOUBLE_EQ( 3.75, (double) C[4](0) );
	ASSERT_DOUBLE_EQ( 4.75, (double) C[4](1) );
	ASSERT_DOUBLE_EQ( 4.75, (double) C[5](0) );
	ASSERT_DOUBLE_EQ( 3.75, (double) C[5](1) );
	ASSERT_DOUBLE_EQ( 5.75, (double) C[6](0) );
	ASSERT_DOUBLE_EQ( 2.75, (double) C[6](1) );
}

TEST(ICP, calculateAffineTransformation) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d C;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	Eigen::MatrixXd c(7,2);
	c << -1.19059, 2.15594,-0.299505,3.24505,0.591584,4.33416,1.48267,5.42327,3.75, 4.75,4.75, 3.75,5.75, 2.75;
	for(int i=0;i<c.rows();i++)
		C.push_back((c.block<1,2>(i,0)).transpose());
	const Eigen::Matrix3d A = ICP::calculateAffineTransformation(Q,C);
	
	if (A.isZero(1e-3)) {
		FAIL() << "The method returned the zero matrix.";
	}
	if (A.isIdentity(1e-3)) {
		FAIL() << "The method returns the identity matrix.";
	}

	Eigen::Matrix3d solution;
	solution <<  0.993669,  0.112347, -0.529593,
			    -0.112347,  0.993669, -0.224951,
				        0,        0,         1;

	if (A.isApprox(solution.inverse(), 0.001)) {
		FAIL() << "The method returned an inverted transformation. Make sure you got the order of variables correct.";
	}
	if (A.topLeftCorner(2, 2).isApprox(solution.topLeftCorner(2, 2).transpose(), 0.001)) {
		FAIL() << "The method returns a transformation with inverted rotation component. Make sure you got the order of U and V correct.";
	}
	if (A.topLeftCorner(2, 2).isIdentity(0.001)) {
		FAIL() << "The method returns a transformation with the identity rotation.";
	}

	if (A.topRightCorner(2, 1).isZero(0.001)) {
		FAIL() << "The method returns a transformation matrix with zero translation. Make sure you translate the point clouds so that their centers of mass match.";
	}

	ASSERT_NEAR((double) solution(0,0), (double) A(0,0), 0.001);
	ASSERT_NEAR((double) solution(0,1), (double) A(0,1), 0.001);
	ASSERT_NEAR((double) solution(0,2), (double) A(0,2), 0.001);
	ASSERT_NEAR((double) solution(1,0), (double) A(1,0), 0.001);
	ASSERT_NEAR((double) solution(1,1), (double) A(1,1), 0.001);
	ASSERT_NEAR((double) solution(1,2), (double) A(1,2), 0.001);
	ASSERT_NEAR((double) solution(2,0), (double) A(2,0), 0.001);
	ASSERT_NEAR((double) solution(2,1), (double) A(2,1), 0.001);
	ASSERT_NEAR((double) solution(2,2), (double) A(2,2), 0.001);
}

TEST(ICP, applyTransformation) 
{
	StdVectorOfVector2d P;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());
	Eigen::Matrix3d A;
	A<< 0.997923, -0.0644194,  0.128267, 0.0644194,  0.997923, -0.615596, 0, 0, 1;
	
	const StdVectorOfVector2d V = ICP::applyTransformation(A,P);

	if (V.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(p.rows(), V.size());
	ASSERT_NEAR(-0.56, (double) V[0](0), 0.01);
	ASSERT_NEAR( 2.35, (double) V[0](1), 0.01);
	ASSERT_NEAR( 0.26, (double) V[1](0), 0.01);
	ASSERT_NEAR( 3.50, (double) V[1](1), 0.01);
	ASSERT_NEAR( 1.09, (double) V[2](0), 0.01);
	ASSERT_NEAR( 4.66, (double) V[2](1), 0.01);
	ASSERT_NEAR( 1.92, (double) V[3](0), 0.01);
	ASSERT_NEAR( 5.81, (double) V[3](1), 0.01);
	ASSERT_NEAR( 2.87, (double) V[4](0), 0.01);
	ASSERT_NEAR( 4.97, (double) V[4](1), 0.01);
	ASSERT_NEAR( 3.83, (double) V[5](0), 0.01);
	ASSERT_NEAR( 4.13, (double) V[5](1), 0.01);
	ASSERT_NEAR( 4.79, (double) V[6](0), 0.01);
	ASSERT_NEAR( 3.29, (double) V[6](1), 0.01);
}

TEST(ICP, computeError) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d C;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	Eigen::MatrixXd c(7,2);
	c << -1.19059, 2.15594,-0.299505,3.24505,0.591584,4.33416,1.48267,5.42327,3.75, 4.75,4.75, 3.75,5.75, 2.75;
	for(int i=0;i<c.rows();i++)
		C.push_back((c.block<1,2>(i,0)).transpose());

	Eigen::Matrix3d A;
	A<< 0.997923, -0.0644194,  0.128267, 0.0644194,  0.997923, -0.615596, 0, 0, 1;
	
	const double error = ICP::computeError(Q,C,A);
	const double epsilon = 0.01;
	ASSERT_GE(error, 0.0);
	ASSERT_NEAR(3.33, error, epsilon);
}

TEST(ICP, iterateOnce)
{
	StdVectorOfVector2d Q;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	
	StdVectorOfVector2d P;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());
	bool flag = false;

	StdVectorOfVector2d P1_0 = ICP::iterateOnce(Q,P,flag,0,6);
	
	if (P1_0.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(P.size(), P1_0.size());
	ASSERT_NEAR(-0.665473, (double) P1_0[0](0), 0.01);
	ASSERT_NEAR( 2.79358, (double) P1_0[0](1), 0.01);
	ASSERT_NEAR( 0.378424, (double) P1_0[1](0), 0.01);
	ASSERT_NEAR( 3.75809, (double) P1_0[1](1), 0.01);
	ASSERT_NEAR( 1.42232, (double) P1_0[2](0), 0.01);
	ASSERT_NEAR( 4.7226, (double) P1_0[2](1), 0.01);
	ASSERT_NEAR( 2.46622, (double) P1_0[3](0), 0.01);
	ASSERT_NEAR( 5.68711, (double) P1_0[3](1), 0.01);
	ASSERT_NEAR( 3.23266, (double) P1_0[4](0), 0.01);
	ASSERT_NEAR( 4.67096, (double) P1_0[4](1), 0.01);
	ASSERT_NEAR( 3.9991, (double) P1_0[5](0), 0.01);
	ASSERT_NEAR( 3.65481, (double) P1_0[5](1), 0.01);
	ASSERT_NEAR( 4.76555, (double) P1_0[6](0), 0.01);
	ASSERT_NEAR( 2.63866, (double) P1_0[6](1), 0.01);
	ASSERT_TRUE(flag);

	flag = false;

	StdVectorOfVector2d P1_1 = ICP::iterateOnce(Q,P,flag,1,6);

	ASSERT_EQ(P.size(), P1_1.size());
	ASSERT_NEAR(-0.665473, (double) P1_0[0](0), 0.01);
	ASSERT_NEAR( 2.79358, (double) P1_0[0](1), 0.01);
	ASSERT_NEAR( 0.378424, (double) P1_0[1](0), 0.01);
	ASSERT_NEAR( 3.75809, (double) P1_0[1](1), 0.01);
	ASSERT_NEAR( 1.42232, (double) P1_0[2](0), 0.01);
	ASSERT_NEAR( 4.7226, (double) P1_0[2](1), 0.01);
	ASSERT_NEAR( 2.46622, (double) P1_0[3](0), 0.01);
	ASSERT_NEAR( 5.68711, (double) P1_0[3](1), 0.01);
	ASSERT_NEAR( 3.23266, (double) P1_0[4](0), 0.01);
	ASSERT_NEAR( 4.67096, (double) P1_0[4](1), 0.01);
	ASSERT_NEAR( 3.9991, (double) P1_0[5](0), 0.01);
	ASSERT_NEAR( 3.65481, (double) P1_0[5](1), 0.01);
	ASSERT_NEAR( 4.76555, (double) P1_0[6](0), 0.01);
	ASSERT_NEAR( 2.63866, (double) P1_0[6](1), 0.01);
	ASSERT_TRUE(flag);
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
