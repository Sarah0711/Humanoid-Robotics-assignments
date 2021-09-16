#ifndef PATH_PLANNING_HEURISTIC_H_
#define PATH_PLANNING_HEURISTIC_H_

#include <ara_star/AbstractNode.h>

namespace ara_star {

/**
 * @brief Abstract superclass for all heuristics.
 */
class Heuristic {
public:
	Heuristic() {};
	virtual ~Heuristic() {};

	/**
	 * @brief Returns the heuristic value for a given current node and goal node.
	 * @param currentNode The node that is currently being expanded.
	 * @param goalNode The goal node where the robot is supposed to drive to.
	 * @return The heuristic value.
	 */
	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const = 0;
};

}



#endif /* PATH_PLANNING_HEURISTIC_H_ */
