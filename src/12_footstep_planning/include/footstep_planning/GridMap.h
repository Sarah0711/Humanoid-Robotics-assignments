#ifndef PATH_PLANNING_GRIDMAP_H_
#define PATH_PLANNING_GRIDMAP_H_

#include <stdexcept>
#include <vector>

namespace footstep_planning {

/**
 * @brief Represents a grid map.
 */
struct GridMap {
public:
	const size_t width;   ///< The width of the map.
	const size_t height;  ///< The height of the map.
	/**
	 * @brief Tests if a grid cell is occupied.
	 * @param x The x coordinate of the grid cell.
	 * @param y The y coordinate of the grid cell.
	 * @return True iff the grid cell is occupied.
	 * @throws std::out_of_range The cell index is outside the map bounds.
	 */
	bool isOccupied(const int& x, const int& y) const {
		if (x < 0 || x >= static_cast<int>(width) || y < 0 || y >= static_cast<int>(width)) {
			throw std::out_of_range("Index out of bounds in call to isOccupied()");
		}
		return data[y * width + x];
	}

	/**
	 * @brief Constructs a grid map from given data.
	 * @param width The width of the map.
	 * @param height The height of the map.
	 * @param data The occupancy data in row-major order.
	 */
	GridMap(const size_t& width, const size_t& height, const std::vector<bool>& data) : width(width), height(height), data(data) {};

private:
	const std::vector<bool> data;
};

}  // namespace footstep_planning

#endif /* PATH_PLANNING_GRIDMAP_H_ */
