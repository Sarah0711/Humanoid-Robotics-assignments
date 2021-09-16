#ifndef RRT_GRIDMAP_H_
#define RRT_GRIDMAP_H_

#include <stdexcept>
#include <vector>

namespace rrt {

/**
 * @brief Grid map.
 */
struct GridMap {
public:
	const size_t width;    ///< Width of the grid map.
	const size_t height;   ///< Height of the grid map.
	 /**
	  * @brief Tests if a grid cell is occupied.
	  * @param x x coordinate of the cell.
	  * @param y y coordinate of the cell.
	  * @return True iff the cell is occupied.
	  * @throws std::out_of_range if the index is out of bounds.
	  */
	inline bool isOccupied(const int& x, const int& y) const {
		if (x < 0 || x >= static_cast<int>(width) || y < 0 || y >= static_cast<int>(width)) {
			throw std::out_of_range("Index out of bounds in call to isOccupied()");
		}
		const bool result = data[y * width + x];
		return result;
	}

	/**
	 * Constructs a new grid map.
	 * @param width Width of the grid map in cells.
	 * @param height Height of the grid map in cells.
	 * @param data Occupancy data for each cell.
	 */
	GridMap(const size_t& width, const size_t& height, std::vector<bool>& data) :
			width(width), height(height), data(data) {
		if (data.size() != width * height) {
			throw std::invalid_argument("The size of the data vector does not match the number of cells according to width and height.");
		}
	}

private:
	std::vector<bool> data;
};

}  // namespace rrt

#endif /* RRT_GRIDMAP_H_ */
