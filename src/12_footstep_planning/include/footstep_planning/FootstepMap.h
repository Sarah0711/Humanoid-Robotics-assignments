#ifndef PATH_PLANNING_FOOTSTEPMAP_H_
#define PATH_PLANNING_FOOTSTEPMAP_H_

#include <vector>
#include <limits>
#include <footstep_planning/GridMap.h>
#include <angles/angles.h>

namespace footstep_planning {

/**
 * @brief Represents a grid map with additional methods for footstep planning.
 */
struct FootstepMap : GridMap {
public:
	/**
	 * Constructs a grid map for footstep planning.
	 * @param width The width of the map in cells.
	 * @param height The height of the map in cells.
	 * @param data The occupancy data in row-major order.
	 * @param distanceMaps A vector of distance maps for fast distance to obstacle lookups.
	 * @param resolution The size of a grid cell in meters.
	 * @param scale The multiplication factor for the values of the distance maps.
	 */
	FootstepMap(const size_t& width, const size_t& height, const std::vector<bool>& data,
			const std::vector<std::vector<unsigned char> >& distanceMaps,
			const double& resolution, const double& scale);
	virtual ~FootstepMap();

	/**
	 * Returns the approximate distance to the nearest obstacle.
	 * @param x Foot position in x direction in meters.
	 * @param y Foot position in y direction in meters.
	 * @param theta Foot orientation in radians.
	 * @return The distance to the nearest obstacle in meters.
	 */
	double getDistanceToNearestObstacle(const double& x, const double& y, const double& theta) const;

private:
	const std::vector<std::vector<unsigned char> > distanceMaps;
	const double resolution;
	const double scale;
};

}  // namespace footstep_planning

#endif /* PATH_PLANNING_FOOTSTEPMAP_H_ */
