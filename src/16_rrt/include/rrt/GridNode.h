#ifndef RRT_GRIDNODE_H_
#define RRT_GRIDNODE_H_

#include <map>
#include <sstream>
#include <string>
#include <rrt/AbstractNode.h>

namespace rrt {

/**
 * @brief Subclass of AbstractNode representing a grid node in an RRT tree.
 *
 * This class can only be instantiated via the GridNode::get(const int&, const int&)
 * static method. The nodes are created lazily on request and stored in a hash map
 * for efficiency reasons in sparse maps.
 */
class GridNode: public AbstractNode {
public:
	const int x;  ///< The x coordinate in grid cells.
	const int y;  ///< The y coordinate in grid cells.

	/**
	 * @brief Returns a pointer to the grid cell node with given coordinates.
	 * @param x The x coordinate in grid cells.
	 * @param y The y coordinate in grid cells.
	 * @return A pointer to the grid cell node.
	 *
	 * Use this method to get access to the node representing a grid cell.
	 * The nodes are allocated lazily on request.
	 */
	static GridNode* get(const int& x, const int&y);
	std::string toString() const;
	std::string toLogString() const;

private:
	GridNode(const int& x, const int& y) : x(x), y(y) {}
	typedef std::pair<int, int> IndexType;
	typedef std::map<IndexType, GridNode*> MapType;
	static MapType nodes;
};

/**
 * @brief Hash function for grid nodes.
 * @param node Grid node.
 * @return Hash value for the node.
 */
int hash_value(GridNode &node);

}  // namespace rrt

#endif /* RRT_GRIDNODE_H_ */
