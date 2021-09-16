#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <sstream>

#include <ara_star/AbstractNode.h>
#include <ara_star/GridNode.h>
#include <ara_star/GridMap.h>
#include <ara_star/Heuristic.h>
#include "ClosedList.h"
#include "OpenList.h"

namespace ara_star {

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
 * @brief Abstract superclass for A* planning.
 */
class PathPlanning
{
public:
	PathPlanning() {};
   	virtual ~PathPlanning() {};
   	/**
   	 * @brief Plans a path from a start node to a goal node.
   	 * @param[in] startNode The start node.
   	 * @param[in] goalNode The goal node.
   	 * @return The optimal path from the start node to the end node.
   	 */
   	virtual std::deque<const AbstractNode*> planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode);

   	/**
   	 * @brief Extracts the path from the currentNode back to the start node.
   	 * @param[in] node The current node.
   	 * @return The path from the start node up to the current node.
   	 */
   	virtual std::deque<const AbstractNode*> followPath(const AbstractNode * const node);

   	/**
   	 * @brief Expands the current node and adds its neighbor cells to the list of open cells.
   	 * @param[in] currentNode The current node.
   	 * @param[in] goalNode The goal node where the robot should travel to.
   	 * @param[in,out] openList The list of open nodes.
   	 * @param[in] closedList The list of nodes that are already visited and closed by the algorithm.
   	 */
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
	/**
	 * @brief Calculates the costs for traveling from currentNode to successorNode.
	 * @param[in] currentNode The current grid cell.
	 * @param[in] successorNode The next grid cell where the robot will travel to.
	 * @return The costs (i.e., the Euclidean distance) for traveling from currentNode to successorNode.
	 */
	double getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const;

	/**
	 * @brief Returns a vector of neighbor nodes of the current node where the robot can travel to.
	 * @param[in] currentNode The current grid cell.
	 * @param[in] map The grid map of the environment.
	 * @return A vector of neighbor grid cells that are accessible to the robot.
	 */
	std::vector<AbstractNode *> getNeighborNodes(const GridNode * const currentNode, const GridMap& map);

	/**
	 * @brief Returns true if the current node is close to the goal node.
	 * @param[in] currentNode The current node.
	 * @param[in] goalNode The goal node.
	 * @return True iff the current grid cell equals the goal grid cell.
	 */
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

}  // namespace ara_star


#endif  // PATH_PLANNING_H_
