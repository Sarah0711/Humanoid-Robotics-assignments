#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <sstream>

#include <path_planning/AbstractNode.h>
#include <path_planning/GridNode.h>
#include <path_planning/GridMap.h>
#include <path_planning/Heuristic.h>
#include "ClosedList.h"
#include "OpenList.h"

namespace path_planning {

/**
 * @brief Abstract superclass for all grid-based heuristics.
 *
 * The class overloads the heuristic() function for convenience.
 */
class GridHeuristic : public Heuristic {
public:
	GridHeuristic() {};
	virtual ~GridHeuristic() {};

	/**
	 * Returns the heuristic value for a given current node and goal node.
	 * @param currentNode The node that is currently being expanded.
	 * @param goalNode The goal node where the robot is supposed to drive to.
	 * @return The heuristic value.
	 * @throws std::invalid_argument if one of the arguments is NULL.
	 *
	 * This overloads Heuristic::heuristic() with GridNode parameters for convenience.
	 */
	virtual double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const = 0;

	double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const {
		if (!currentNode) {
			throw std::invalid_argument("GridHeuristic::heuristic(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("GridHeuristic::heuristic(): goalNode is NULL");
		}
		return heuristic(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
	}
};

/**
 * @brief Straight line (= Euclidean distance) heuristic.
 */
class StraightLineDistanceHeuristic : public GridHeuristic {
public:
	StraightLineDistanceHeuristic() {};
	virtual ~StraightLineDistanceHeuristic() {};
	double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const;
};

/**
 * @brief Manhattan distance (= 1-norm metric) heuristic.
 */
class ManhattanDistanceHeuristic : public GridHeuristic {
public:
	ManhattanDistanceHeuristic() {};
	virtual ~ManhattanDistanceHeuristic() {};
	double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const;
};

/**
 * @brief Abstract superclass for A* planning.
 */
class PathPlanning
{
public:
	PathPlanning() {};
   	virtual ~PathPlanning() {};
   	virtual std::deque<const AbstractNode*> planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode);
   	virtual std::deque<const AbstractNode*> followPath(const AbstractNode * const node);
   	virtual void expandNode(const AbstractNode * const currentNode, const AbstractNode * const goalNode,
   			OpenList& openList, const ClosedList& closedList);

   	/**
   	 * @brief Returns a vector of neighbor nodes of the current node where the robot can travel to.
   	 * @param[in] currentNode The current node.
   	 * @return A vector of neighbor nodes that are accessible to the robot.
   	 * @throws std::invalid_argument if currentNode is NULL.
   	 */
   	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) = 0;

   	/**
   	 * @brief Returns the heuristic (wraps an instance of the Heuristic class).
   	 * @param currentNode The node currently being expanded.
   	 * @param goalNode The goal node where the robot is supposed to travel to.
   	 * @return The heuristic value.
	 * @throws std::invalid_argument if one of the arguments is NULL.
   	 */
   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) = 0;

   	/**
   	 * @brief Calculates the costs for traveling from currentNode to successorNode.
   	 * @param[in] currentNode The current node.
   	 * @param[in] successorNode The next node where the robot will travel to.
   	 * @return The costs (e.g., the Euclidean distance) for traveling from currentNode to successorNode.
	 * @throws std::invalid_argument if one of the arguments is NULL.
   	 */
	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const = 0;

	/**
	 * @brief Returns true if the current node is close to the goal node.
	 * @param[in] currentNode The current node.
	 * @param[in] goalNode The goal node.
	 * @return True iff the current node is close enough to the goal node.
	 * @throws std::invalid_argument if one of the arguments is NULL.
	 */
	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) = 0;
};

/**
 * @brief Grid-based A* planning.
 *
 * This class overloads the methods of PathPlanning with grid-based node types for convenience.
 */
class GridPathPlanning : public PathPlanning {
public:
	/**
	 * @brief Constructs an A* planner for a given grid map and heuristic.
	 * @param map The grid map for which a plan should be generated.
	 * @param heuristic The heuristic to use.
	 */
	GridPathPlanning(const GridMap& map, const GridHeuristic& heuristic) : map_(map), heuristic_(heuristic) {};
	virtual ~GridPathPlanning() {};

	// Overloaded methods for convenience
	double getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const;
	std::vector<AbstractNode *> getNeighborNodes(const GridNode * const currentNode, const GridMap& map);
	bool isCloseToGoal(const GridNode * const currentNode, const GridNode * const goalNode);

	// Implementation of abstract superclass methods
   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
		if (!currentNode) {
			throw std::invalid_argument("GridPathPlanning::heuristic(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("GridPathPlanning::heuristic(): goalNode is NULL");
		}
   		return heuristic_.heuristic(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
   	}
	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const {
		if (!currentNode) {
			throw std::invalid_argument("GridPathPlanning::getCosts(): currentNode is NULL");
		}
		if (!successorNode) {
			throw std::invalid_argument("GridPathPlanning::getCosts(): successorNode is NULL");
		}
		return getCosts(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(successorNode));
	}
	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) {
		if (!currentNode) {
			throw std::invalid_argument("GridPathPlanning::getNeighborNodes(): currentNode is NULL");
		}
		return getNeighborNodes(static_cast<const GridNode *>(currentNode), map_);
	}
	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
		if (!currentNode) {
			throw std::invalid_argument("GridPathPlanning::isCloseToGoal(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("GridPathPlanning::isCloseToGoal(): goalNode is NULL");
		}
		return isCloseToGoal(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
	}


private:
   	const GridMap map_;
   	const GridHeuristic& heuristic_;
};

}  // namespace path_planning


#endif  // PATH_PLANNING_H_
