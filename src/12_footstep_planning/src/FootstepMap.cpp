#include <vector>
#include <limits>
#include <angles/angles.h>
#include <footstep_planning/GridMap.h>
#include <footstep_planning/FootstepMap.h>

namespace footstep_planning {
FootstepMap::FootstepMap(const size_t& width, const size_t& height,
		const std::vector<bool>& data,
		const std::vector<std::vector<unsigned char> >& distanceMaps,
		const double& resolution, const double& scale) :
		GridMap(width, height, data), distanceMaps(distanceMaps),
		resolution(resolution), scale(scale) {
}

FootstepMap::~FootstepMap() {
}


double FootstepMap::getDistanceToNearestObstacle(const double& x, const double& y, const double& theta) const {
	double t = angles::to_degrees(angles::normalize_angle_positive(theta));
	if (t >= 180.) {
		t -= 180.;
	}
	const size_t ts = static_cast<size_t>(floor(t / 15. + 0.5));
	const size_t xs = static_cast<size_t>(floor(x / resolution + 0.5));
	const size_t ys = height - static_cast<size_t>(floor(y / resolution + 0.5));
	double distance;
	if (ts < distanceMaps.size() && xs < width && ys < height) {
		const unsigned char val = distanceMaps[ts][ys * width + xs];
		distance = static_cast<double>(val) / scale * resolution;
	} else {
		distance = 0.0;
	}
	return distance;
}
}  // namespace footstep_planning
