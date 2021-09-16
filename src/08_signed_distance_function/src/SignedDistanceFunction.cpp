#include <signed_distance_function/SignedDistanceFunction.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

namespace signed_distance_function {

/**
 * \brief Calculates the Euclidean distance between two 2D points.
 * \param[in] pointA The first point.
 * \param[in] pointB The second point.
 * \return The Euclidean distance
 */
double SignedDistanceFunction::calculateDistance(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB) {
	double distance = 0.0;
	//TODO: Implement the Euclidean distance.
	distance = sqrt(pow((pointA(0)-pointB(0)), 2) + pow(pointA(1)-pointB(1),2));
	return distance;
}

/**
 * \brief Truncates the signed distance.
 * \param[in] signedDistance The signed distance in cells (negative outside the object, positive inside the object).
 * \param[in] delta The truncating limit in cells.
 * \return The truncated signed distance.
 */
double SignedDistanceFunction::truncateDistance(const double& signedDistance, const double& delta) {
	double truncatedDistance = 0.0;
	//TODO: Implement the truncated signedDistance function.
	// Note that signedDistance can be negative here.
	if (signedDistance <= -delta)
	{
		truncatedDistance = -delta;
	}
	else if (abs(signedDistance) < delta)
	{
		truncatedDistance = signedDistance;
	}
	else if (signedDistance >= delta)
	{
		truncatedDistance = delta;
	}

	return truncatedDistance;
}

/**
 * \brief Calculates the weight according to the signed distance.
 * \param[in] signedDistance The signed distance in cells (negative outside the object, positive inside the object).
 * \param[in] delta The truncation limit.
 * \param[in] epsilon The lower limit.
 * \return The weight.
 */
double SignedDistanceFunction::calculateWeight(const double& signedDistance, const double& delta, const double& epsilon) {
	double weight = 0.0;
	//TODO calculate the weight according to the current measurement
	if (signedDistance < epsilon)
	{
		weight = 1;
	}
	else if (signedDistance >= epsilon && signedDistance <= delta)
	{
		weight = (delta - signedDistance) / (delta - epsilon);
	}
	else if (signedDistance > delta)
	{
		weight = 0;
	}
	return weight;
}

/**
 * \brief Update the signed distance value in the map
 * \param[in] signedDistance The new signed distance of the current measurement.
 * \param[in] weight The new weight of the current measurement.
 * \param[in] oldSignedDistance The old signed distance contained in the map.
 * \param[in] oldWeight The old weight of the map cell.
 * \return The new signed distance entry of the map cell.
 */
double SignedDistanceFunction::updateMap(
		const double& signedDistance, const double& weight,
		const double& oldSignedDistance, const double& oldWeight) {
	double newSignedDistance = 0.0;
	//TODO calculate the new signed distance that will be written into the map
	newSignedDistance = ((oldWeight * oldSignedDistance) + (weight * signedDistance)) / (oldWeight + weight);
	return newSignedDistance;
}

/**
 * \brief Update the weight of the map cell
 * \param[in] weight The new weight of the current measurement.
 * \param[in] oldWeight The old weight of the map cell.
 * \return The new weight of the map cell.
 */
double SignedDistanceFunction::updateWeight(const double& weight, const double& oldWeight) {
	double newWeight = 0.0;
	//TODO calculate the new weight of the map cell
	newWeight = weight + oldWeight;
	return newWeight;
}

/**
 * \brief Adds the information from the new laser scan to the map.
 * \param[in,out] map The map containing the signed distance value for each cell
 * \param[in,out] weights The weight of each map cell.
 * \param[in] measurement The current laser measurement.
 */
void SignedDistanceFunction::integrateLaserScan(Eigen::MatrixXd& map, Eigen::MatrixXd& weights, const Measurement& measurement) {
	const double delta = 5.;
	const double epsilon = 1.;
	double distance = 0.0;
	double weight = 0.0;
	int l;
	//TODO: Add the information from the new laser scan to the map.
	Eigen::Vector2d lp_begin;

	Eigen::Vector2d returned_point_begin;
	double signed_distance;
	double old_distance;
	double old_weight;
	VectorOfPoints lp_vector;
	VectorOfPoints points;
	lp_vector = measurement.laserPoints;
	
	for (int i = 0; i < measurement.laserPoints.size(); i++)
	{
		lp_begin = lp_vector[i];
		points = bresenham(measurement.robotPosition,lp_begin, map.rows(), map.cols());
		for (int k = 0; k < points.size(); ++k)
		{
			returned_point_begin = points[k];
			distance = calculateDistance(lp_begin,returned_point_begin);
			if (lp_begin(0) > measurement.robotPosition(0))
			{
				if (returned_point_begin(0) < lp_begin(0))
				{
					distance = -1 * distance;
				}
			}
			else if (lp_begin(0) == measurement.robotPosition(0))
			{
				if (lp_begin(1) <= measurement.robotPosition(1))
				{
					if (returned_point_begin(1) > lp_begin(1))
					{
						distance = -1 * distance;
					}
				}
				else
				{
					if (returned_point_begin(1) < lp_begin(1))
					{
						distance = -1 * distance;
					}
				}
			}
			else
			{
				if (returned_point_begin(0) > lp_begin(0))
				{
					distance = -1 * distance;
				}
			}
			signed_distance = truncateDistance(distance, delta);
			weight = calculateWeight(signed_distance, delta, epsilon);
			
			
			if (weight > 0)
			{
				old_distance = map(int(returned_point_begin(1)), int(returned_point_begin(0)));
				old_weight = weights(int(returned_point_begin(1)), int(returned_point_begin(0)));
				map(int(returned_point_begin(1)), int(returned_point_begin(0))) = updateMap(signed_distance,weight,old_distance,old_weight);
				weights(int(returned_point_begin(1)), int(returned_point_begin(0))) = updateWeight(weight, old_weight);
			}

		}

				
	}

	
}

/**
 * \brief Modified version of the Bresenham algorithm for calculating points on a straight line.
 * \param[in] pointA The first end point of the line
 * \param[in] pointB The second end point of the line
 * \param[in] numRows The number of rows of the map.
 * \param[in] numCols The number of columns of the map.
 * \return Vector of all points between A and B+(B-A)
 *
 * This method is based on the code on http://de.wikipedia.org/wiki/Bresenham-Algorithmus
 */
VectorOfPoints SignedDistanceFunction::bresenham(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB,
		const size_t& numRows, const size_t& numCols) {

	if (pointA.x() < 0 || pointA.x() > numCols || pointA.y() < 0 || pointA.y() > numRows) {
		std::stringstream ss;
		ss << "Error in bresenham(): pointA with coordinates (" << pointA.x() << ", " << pointA.y() << ") is outside the map";
		throw std::invalid_argument(ss.str());
	}
	if (pointB.x() < 0 || pointB.x() > numCols || pointB.y() < 0 || pointB.y() > numRows) {
		std::stringstream ss;
		ss << "Error in bresenham(): pointB with coordinates (" << pointB.x() << ", " << pointB.y() << ") is outside the map";
		throw std::invalid_argument(ss.str());
	}

	VectorOfPoints pointsOnLine;

	const Eigen::Vector2d pointC = pointB + (pointB - pointA);

	int x0 = static_cast<int>(pointA.x());
	int y0 = static_cast<int>(pointA.y());
	const int x1 = static_cast<int>(pointC.x());
	const int y1 = static_cast<int>(pointC.y());
	const int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
	const int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
	int err = dx+dy, e2; /* error value e_xy */

	Eigen::Vector2d p;
	while(true) {
		p << x0, y0;
		if (x0 >= 0 && x0 < static_cast<int>(numCols) && y0 >= 0 && y0 < static_cast<int>(numRows)) {
			pointsOnLine.push_back(p);
		}
		if (x0==x1 && y0==y1) break;
		e2 = 2*err;
		if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
		if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
	}

	return pointsOnLine;
}



}  // namespace signed_distance_function
