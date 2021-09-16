#ifndef PATH_PLANNING_GRIDNODE_H_
#define PATH_PLANNING_GRIDNODE_H_

#include <map>
#include <sstream>
#include <string>
#include <ara_star/AbstractNode.h>

namespace ara_star {

/**
 * @brief Represents a grid node for grid-based A* planning.
 *
 * The grid cells are stored in a sparse hash map for efficiency in large maps.
 */
class GridNode : public AbstractNode {
public:
	const int x;  ///< The x coordinate of the grid cell.
	const int y;  ///< The y coordinate of the grid cell.

	/**
	 * @brief Provides access to a grid cell.
	 * @param x ///< The x coordinate of the grid cell.
	 * @param y ///< The y coordinate of the grid cell.
	 * @return The grid cell.
	 *
	 * The grid cell is allocated lazily on request.
	 */
	static GridNode* get(const int& x, const int&y);
	std::string toString() const;
	std::string toLogString() const;

private:
	GridNode(const int& x, const int& y) : x(x), y(y) {};
	typedef std::pair<int, int> IndexType;
	typedef std::map<IndexType, GridNode*> MapType;
	static MapType nodes;
};

int hash_value(const GridNode &node);

}  // namespace ara_star


#endif /* PATH_PLANNING_GRIDNODE_H_ */
