#include <icp/ICP.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <Eigen/SVD> 

using namespace std;
namespace icp
{
/**
 * \brief Compute the Euclidean distance between a pair of 2D points.
 * \param[in] p1: The first 2D point.
 * \param[in] p2: The second 2D point.
 * \return The Euclidean distance between the two input 2D points.
 */
	double ICP::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
	{
		double result = -1.0;
		//TODO: Calculate the distance between the points.
		result = sqrt(pow((p1(0) - p2(0)), 2) + pow(p1(1) - p2(1), 2));
		return result;
	}
/**
 * \brief Compute the closest point that lies on a given line to a given 2D point.
 * \param[in] pX: The given 2D point, to which we need to compute the closest point that lies on a line.
 * \param[in] pL1: A point that lies on the line.
 * \param[in] pL2: Another point that lies on the line.
 * \return The closest point on the line.
 */
	Eigen::Vector2d ICP::closestPointOnLine(const Eigen::Vector2d& pX, const Eigen::Vector2d& pL1, const Eigen::Vector2d& pL2)
	{
		Eigen::Vector2d result(0., 0.);
		if (pL1(0) == pL2(0))
		{
			result(0) = pL1(0);
			result(1) = pX(1);
			return result;
		}
		if (pL1(1) == pL2(1))
		{
			result(0) = pX(0);
			result(1) = pL1(1);
			return result;
		}
		double s1, c1,s2,c2;
		//Original line :
		//y = s1 * x + c1
		s1 = (pL2(1) - pL1(1)) / (pL2(0) - pL1(0));
		c1 = pL1(1) - s1 * pL1(0);

		//Perpendicular line :
		//y = s2 * x + c2
		s2 = -1 / s1;
		c2 = pX(1) - s2 * pX(0);

		result(0) = -(c1 - c2) / (s1 - s2);
		result(1) = s1 * result(0) + c1;
		return result;

	} 
/**
 * \brief Get the minimum value within vector.
 * \param[in] dist: A vector of values.
 * \return The minimum value.
 */
	double ICP::minDistance(const std::vector<double>& dist)
	{
		double result = 0.0;
		//double min = DBL_MAX;
		double min = 100000000.0;
		//TODO: Find and return the minimum value in the vector dist
		for (int i = 0; i < dist.size(); i++)
		{
			if (dist[i]<min)
			{
				min = dist[i];
			}
		}
		result = min;
		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'closest point' matching method.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 * The result vector will have the same length as Q.
 */
	StdVectorOfVector2d ICP::euclideanCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d result;
		double dist = 0.0;
		double min_dist = 0.0;
		Eigen::Vector2d p1;
		Eigen::Vector2d p2;
		Eigen::Vector2d match;
		//TODO: Compute corresponding points using the "closest point" method
		for (int i = 0; i < Q.size(); ++i)
		{
			min_dist = 100000000.0;
			for (int j = 0; j < P.size(); ++j)
			{
				dist = distance(Q[i], P[j]);
				if (dist <= min_dist)
				{
					p2 = P[j];
					min_dist = dist;
				}
			}
			result.push_back(p2);
		}
		
		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'point-to-line' matching method .
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::closestPointToLineCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;
		//TODO: Compute corresponding points using the "point-to-line" method
		double dist = 0.0;
		double min_dist = 0.0;
		double dist1, dist2;
		Eigen::Vector2d p1;
		Eigen::Vector2d p2;
		Eigen::Vector2d match;
		int index;
		//TODO: Compute corresponding points using the "closest point" method
		for (int i = 0; i < Q.size(); ++i)
		{
			min_dist = 100000000.0;
			for (int j = 0; j < P.size(); ++j)
			{
				dist = distance(Q[i], P[j]);
				if (dist <= min_dist)
				{
					match = P[j];
					index = j;
					min_dist = dist;
				}
			}
			if (index == 0)
			{
				p1 = P[index];
				p2 = P[index + 1];
			}
			else if (index == P.size())
			{
				p1 = P[index];
				p2 = P[index - 1];
			}
			else
			{
				dist1 = distance(Q[i], P[index - 1]);
				dist2 = distance(Q[i], P[index + 1]);
				if (dist1 <= dist2)
				{
					p1 = P[index];
					p2 = P[index - 1];
				}
				else
				{
					p1 = P[index];
					p2 = P[index + 1];
				}
			}
			result.push_back(closestPointOnLine(Q[i],p1,p2));
		}


		return result;

	}
/**
 * \brief Compute the affine transformation matrix needed to align the previously computed corresponding points (list C) to the points of list Q.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points, that corresponds to points in list Q.
 * \return An Affine transformation matrix.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	Eigen::Matrix3d ICP::calculateAffineTransformation(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C)
	{
		Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
		//TODO: Compute the affine transformation matrix
		//calculate mean for points in Q vector
		Eigen::Vector2d Q_,C_;
		Q_<<0,0;
		C_<<0,0;

		Eigen::Vector2d point;
		for (int i = 0; i < Q.size(); ++i)
		{
			Q_ += Q[i];
		}
		Q_ = Q_ / Q.size();

		//calculate mean for points in C vector
		for (int i = 0; i < C.size(); ++i)
		{
			C_ += C[i];
		}

		C_ = C_ / C.size();

		//calculate new sets of points by subtracting mean from all of them
		StdVectorOfVector2d newQ;
		StdVectorOfVector2d newC;
		for (int i = 0; i < Q.size(); ++i)
		{
			newQ.push_back(Q[i] - Q_);
		}
		for (int i = 0; i < C.size(); ++i)
		{
			newC.push_back(C[i] - C_);
		}

		//now calculate W
		Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
		Eigen::Vector2d point2;
		for (int i = 0; i < C.size(); ++i)
		{
			point = newC[i];
			point2 = newQ[i];
			W = W + (point * point2.transpose());
		}
		//Calculate SVD decomposition
		Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
		Eigen::Matrix2d U;
		Eigen::Matrix2d V;
		U = svd.matrixU();
		V = svd.matrixV();

		//calculate rotation and translation matrix
		Eigen::Matrix2d R;
		Eigen::Vector2d t;
		R =( U*(V.transpose())).transpose();

		t = Q_ - (R*C_);
		result(0,0) = R(0,0);
		result(0, 1) = R(0, 1);
		result(0, 2) = t(0);
		result(1, 0) = R(1, 0);
		result(1, 1) = R(1, 1);
		result(1, 2) = t(1);
		result.row(2) << 0, 0, 1;
		return result;

	}
/**
 * \brief Apply the affine transformation matrix on the points in list P.
 * \param[in] A: Affine transformation matrix.
 * \param[in] P: A vector of 2D points, on which the affine transformation will be applied.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::applyTransformation(const Eigen::Matrix3d& A, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;
		Eigen::Vector2d point;
		Eigen::Vector2d r;
		Eigen::Vector3d pm= Eigen::Vector3d::Zero(3);
		Eigen::Vector3d pr = Eigen::Vector3d::Zero(3);
		//TODO: Apply the affine transformation A to the points in list P.
		for (int i = 0; i < P.size(); ++i)
		{
			point = P[i];
			pm<<point(0), point(1), 1;
			pr = A * pm;
			r(0) = pr(0);
			r(1) = pr(1);
			result.push_back(r);
		}

		return result;
	}
/**
 * \brief Compute the error between the points in Q list and the transformed corresponding points.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points corresponding to point in Q.
 * \param[in] A: Affine transformation matrix.
 * \return The error of applying the transformation.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	double ICP::computeError(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C, const Eigen::Matrix3d& A)
	{
		double result = 0.0;
		//TODO: Compute the error after the transformation.
		StdVectorOfVector2d C_transformed = applyTransformation(A,C);

		for(int i=0; i<Q.size();i++){
			result += pow(distance(Q[i],C_transformed[i]),2);

		}
		return result;		
	}


/**
 * \brief Perform one iteration of ICP and prints the error of that iteration.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points, that we need to transform them to be aligned with points in Q list.
 * \param[out] convergenceFlag: A flag passed by reference to determine whether the alignment error has crossed the convergence threshold or not. The flag should be set to 'true' in case of convergence.
 * \param[in] pointToLineFlag: A flag that states the matching method to be used. Its value is set to 'true' when point-to-line method is needed, and to 'false' when closest point method is needed.
 * \param[in] threshold: The maximum value of acceptable error.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::iterateOnce(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P, bool & convergenceFlag, const bool pointToLineFlag, const double threshold)
	{
		StdVectorOfVector2d result;
		
		StdVectorOfVector2d closest;

		if(pointToLineFlag){
			closest = closestPointToLineCorrespondences(Q,P);
		}
		else{
			closest = euclideanCorrespondences(Q,P);
		}
		
		double error = computeError(Q, closest, calculateAffineTransformation(Q,closest));

		result = applyTransformation(calculateAffineTransformation(Q,closest),P);

		if (error <= threshold) {
			convergenceFlag = true;
		}

		return result;
	}


}
