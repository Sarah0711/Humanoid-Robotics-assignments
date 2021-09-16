#ifndef FOOTSTEP_PLANNING_H_
#define FOOTSTEP_PLANNING_H_

#include <footstep_planning/PathPlanning.h>
#include <footstep_planning/FootstepPlanning.h>
#include <footstep_planning/FootstepMap.h>
#include <footstep_planning/FootstepNode.h>
#include <angles/angles.h>

using namespace footstep_planning;

namespace footstep_planning {

/**
 * @brief Heuristic for footstep planning.
 */
class FootstepHeuristic : public Heuristic {
public:
	FootstepHeuristic() {};
	virtual ~FootstepHeuristic() {};

	double heuristic(const FootstepNode * const currentFootstep, const FootstepNode * const goalFootstep) const;

	/**
	 * @brief Returns the heuristic value for a given current node and goal node.
	 * @param currentNode The node that is currently being expanded.
	 * @param goalNode The goal node where the robot is supposed to drive to.
	 * @return The heuristic value.
	 */
	double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const {
		if (!currentNode) {
			throw std::invalid_argument("FootstepHeuristic::heuristic(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("FootstepHeuristic::heuristic(): goalNode is NULL");
		}
		return heuristic(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(goalNode));
	}
};

/**
 * @brief Footstep-based A* planning.
 *
 * This class overloads the methods of PathPlanning with footstep-based node types for convenience.
 */
class FootstepPlanning : public PathPlanning {
public:
	/**
	 * @brief Footstep action (i.e., a displacement between two footsteps)
	 */
	struct FootstepAction {
		double dx;      ///< The displacement in x direction in meters.
		double dy;      ///< The displacement in y direction in meters.
		double dtheta;  ///< The angular displacement (rotation) in radians.
		Foot foot;      ///< The swing foot (LEFT or RIGHT).

		/**
		 * @brief Creates a new foot step action.
		 * @param dx The displacement in x direction in meters.
		 * @param dy The displacement in y direction in meters.
		 * @param dtheta The angular displacement (rotation) in radians.
		 * @param foot The swing foot (LEFT or RIGHT).
		 */
		FootstepAction(const double& dx, const double& dy, const double& dtheta, const Foot& foot)
		: dx(dx), dy(dy), dtheta(dtheta), foot(foot) {};
	};

	const std::vector<FootstepAction> leftFootActions;   ///< List of available foot step actions for the left foot.
	const std::vector<FootstepAction> rightFootActions;  ///< List of available foot step actions for the left foot.

private:
	std::vector<FootstepAction> createFootstepActions(const Foot& foot) {
		const double c = (foot == RIGHT ? 1.0 : -1.0);
		std::vector<FootstepAction> result;
		result.push_back(FootstepAction( 0.00,  c * 0.16,   c * 0.00,  foot));
		result.push_back(FootstepAction( 0.08,  c * 0.09,   c * 0.00,  foot));
		result.push_back(FootstepAction(-0.04,  c * 0.09,   c * 0.00,  foot));
		result.push_back(FootstepAction( 0.00,  c * 0.12,   c * 0.00,  foot));
		result.push_back(FootstepAction( 0.05,  c * 0.14,   c * 0.00,  foot));
		result.push_back(FootstepAction( 0.01,  c * 0.13,  -c * 0.50,  foot));
		result.push_back(FootstepAction( 0.015, c * 0.100,  c * 0.500, foot));
		result.push_back(FootstepAction( 0.04,  c * 0.12,   c * 0.30,  foot));
		result.push_back(FootstepAction(-0.03,  c * 0.12,   c * 0.50,  foot));
		result.push_back(FootstepAction( 0.06,  c * 0.12,   c * 0.00,  foot));
		result.push_back(FootstepAction( 0.04,  c * 0.10,   c * 0.00,  foot));
		result.push_back(FootstepAction(-0.02,  c * 0.12,   c * 0.00,  foot));
		return result;
	}
public:

	/**
     * @brief Constructs an A* planner for a given foot step map.
	 * @param footstepMap
	 */
	FootstepPlanning(const FootstepMap * const footstepMap) :
			leftFootActions(createFootstepActions(LEFT)), rightFootActions(createFootstepActions(RIGHT)),
			footstepMap(footstepMap)
	{
	};
   	virtual ~FootstepPlanning() {};

   	virtual std::vector<AbstractNode *> getNeighborNodes(const FootstepNode * const currentFootstep);
   	virtual double getCosts(const FootstepNode * const currentFootstep, const FootstepNode * const successorFootstep) const;

   	/**
   	 * @brief Returns the heuristic (wraps an instance of the Heuristic class).
   	 * @param currentNode The node currently being expanded.
   	 * @param goalNode The goal node where the robot is supposed to travel to.
   	 * @return The heuristic value.
	 * @throws std::invalid_argument if one of the arguments is NULL.
   	 */
   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
		if (!currentNode) {
			throw std::invalid_argument("FootstepPlanning::heuristic(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("FootstepPlanning::heuristic(): goalNode is NULL");
		}
   		return heuristic_.heuristic(currentNode, goalNode);
   	}

   	/**
   	 * @brief Calculates the costs for traveling from currentNode to successorNode.
   	 * @param[in] currentNode The current node.
   	 * @param[in] successorNode The next node where the robot will travel to.
   	 * @return The costs (e.g., the Euclidean distance) for traveling from currentNode to successorNode.
	 * @throws std::invalid_argument if one of the arguments is NULL.
   	 */
   	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const {
		if (!currentNode) {
			throw std::invalid_argument("FootstepPlanning::getCosts(): currentNode is NULL");
		}
		if (!successorNode) {
			throw std::invalid_argument("FootstepPlanning::getCosts(): successorNode is NULL");
		}
   		return getCosts(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(successorNode));
   	}

   	/**
   	 * @brief Returns a vector of neighbor nodes of the current node where the robot can travel to.
   	 * @param[in] currentNode The current node.
   	 * @return A vector of neighbor nodes that are accessible to the robot.
   	 * @throws std::invalid_argument if currentNode is NULL.
   	 */
   	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) {
		if (!currentNode) {
			throw std::invalid_argument("FootstepPlanning::getNeighborNodes(): currentNode is NULL");
		}
   		return getNeighborNodes(static_cast<const FootstepNode *>(currentNode));
   	}

   	/**
   	 * @brief Tests if a foot step would collide with an occupied cell of the map.
   	 * @param step The foot step.
   	 * @return True iff the foot step collides with an occupied cell.
   	 */
   	virtual bool isColliding(const FootstepNode * const step) {
		if (!step) {
			throw std::invalid_argument("FootstepPlanning::isColliding(): step is NULL");
		}
   		return footstepMap->getDistanceToNearestObstacle(step->x, step->y, step->theta) < 0.01;
   	}

   	virtual bool isCloseToGoal(const FootstepNode * const currentNode, const FootstepNode * const goalNode);

	/**
	 * @brief Returns true if the current node is close to the goal node.
	 * @param[in] currentNode The current node.
	 * @param[in] goalNode The goal node.
	 * @return True iff the current node is close enough to the goal node.
	 * @throws std::invalid_argument if one of the arguments is NULL.
	 */
   	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
		if (!currentNode) {
			throw std::invalid_argument("FootstepPlanning::isCloseToGoal(): currentNode is NULL");
		}
		if (!goalNode) {
			throw std::invalid_argument("FootstepPlanning::isCloseToGoal(): goalNode is NULL");
		}
   		return isCloseToGoal(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(goalNode));
   	}

   	virtual FootstepNode* executeFootstep(const FootstepNode * const currentFootstep, const FootstepAction& action);

protected:
   	FootstepHeuristic heuristic_;          ///< The foot step heuristic.
   	const FootstepMap * const footstepMap; ///< The foot step grid map.
};

}  // namespace footstep_planning

#endif  // FOOTSTEP_PLANNING_H_
