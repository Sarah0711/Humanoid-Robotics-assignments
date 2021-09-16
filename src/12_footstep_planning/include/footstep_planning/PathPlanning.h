#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <sstream>

#include <footstep_planning/AbstractNode.h>
#include <footstep_planning/GridMap.h>
#include <footstep_planning/Heuristic.h>
#include "ClosedList.h"
#include "OpenList.h"

namespace footstep_planning {

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

}  // namespace footstep_planning


#endif  // PATH_PLANNING_H_
