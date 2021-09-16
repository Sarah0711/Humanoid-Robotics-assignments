#ifndef RRT_H_
#define RRT_H_

#include <rrt/GridNode.h>
#include <rrt/GridMap.h>
#include <vector>
#include <deque>

namespace rrt {

/**
 * @brief Abstract superclass for rapidly-exploring random trees.
 */
class RRT {
public:
	/**
	 * @brief Return values for the extendClosestNode method.
	 */
	enum ExtendStepReturnValue {
		REACHED,   ///< Trees connected, path found
		TRAPPED,   ///< Not possible to extend the tree due to collisions or constraints
		EXTENDED   ///< Tree successfully expanded towards the given node
	};

	RRT() : connectionNode(NULL) {}
	virtual ~RRT() {}
	virtual std::deque<AbstractNode *> planPath(AbstractNode * const start, AbstractNode * const goal, const size_t& maxIterations);

	/**
	 * @brief Draws a random grid node that is not occupied and not in the list.
	 * @param[in] list The list of nodes that have already been explored.
	 * @param[in] listGoal The goal of the tree.
	 * @return A random node.
	 */
	virtual AbstractNode * getRandomNode(const std::vector<AbstractNode *>& list, AbstractNode * const listGoal) const = 0;

	/**
	 * @brief Calculates a distance metric between two nodes.
	 * @param[in] node1 The first node.
	 * @param[in] node2 The second node.
	 * @return The distance between the two nodes.
	 */
	virtual double distance(AbstractNode * const node1, AbstractNode * const node2) const = 0;

	virtual AbstractNode * getClosestNodeInList(AbstractNode * const randomNode, const std::vector<AbstractNode *>& list) const;

	virtual ExtendStepReturnValue extendClosestNode(AbstractNode * const randomNode,
			std::vector<AbstractNode *> & list,
			const std::vector<AbstractNode *> & otherList);

	virtual std::deque<AbstractNode *> constructPath(AbstractNode * const connectionNode, AbstractNode * const startNode, AbstractNode * const goalNode) const;
	AbstractNode* tryToConnect(AbstractNode* const currentNode, const std::vector<AbstractNode*>& neighbors,
			 const std::vector<AbstractNode*>& otherList) const;

	/**
	 * @brief Returns the neighbors of a node that are reachable and not already expanded.
	 * @param[in] currentNode The current node.
	 * @param[in] list The list of already expanded nodes in the current tree.
	 */
	virtual std::vector<AbstractNode*> getNeighbors(AbstractNode* const currentNode, const std::vector<AbstractNode*>& list) const = 0;

	virtual void addNearestNeighbor(AbstractNode * const currentNode, std::vector<AbstractNode*>& neighbors, AbstractNode * const randomNode,
			std::vector<AbstractNode*>& list) const;

	/**
	 * @brief Returns the connection node if one has been found
	 * @return The connection node or NULL is none has been found so far.
	 */
	AbstractNode * getConnectionNode() const {
		return connectionNode;
	}

protected:
	AbstractNode * connectionNode;  ///< The connection node set by the extendClosestNode() method.
};

/**
 * @brief Rapidly-exploring random trees for grid maps.
 */
class RRTGrid : public RRT {
public:
	/**
	 * @brief Constructor.
	 * @param map The grid map on which to plan a path.
	 */
	RRTGrid(const GridMap& map) : map(map) {}
	virtual ~RRTGrid() {}

	virtual AbstractNode * getRandomNode(const std::vector<AbstractNode *>& list, AbstractNode * const listGoal) const;

	virtual double distance(GridNode * const node1, GridNode * const node2) const;

	virtual double distance(AbstractNode * const node1, AbstractNode * const node2) const {
		return distance(static_cast<GridNode *>(node1), static_cast<GridNode *>(node2));
	}

	virtual std::vector<AbstractNode*> getNeighbors(GridNode* const currentNode, const std::vector<AbstractNode*>& list) const;

	virtual std::vector<AbstractNode*> getNeighbors(AbstractNode* const currentNode, const std::vector<AbstractNode*>& list) const {
		return getNeighbors(static_cast<GridNode * const>(currentNode), list);
	}


protected:
	const GridMap& map;  ///< The grid map.

};

}  // namespace rrt

#endif  // RRT_H_
