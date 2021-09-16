#ifndef FOOTSTEP_PLANNING_FOOTSTEPNODE_H_
#define FOOTSTEP_PLANNING_FOOTSTEPNODE_H_

#include <footstep_planning/AbstractNode.h>
#include <map>

namespace footstep_planning {

enum Foot { LEFT = 0, RIGHT = 1 };

/**
 * @brief A* node representing a foot step.
 */
class FootstepNode : public AbstractNode {
public:
	double x;       ///< x position in meters
	double y;       ///< y position in meters
	double theta;   ///< orientation in radians
	Foot foot;      ///< foot (LEFT or RIGHT)

	virtual ~FootstepNode();

	/**
	 * @brief Provides access to a foot step node.
	 * @param x The x coordinate of the foot step in meters.
	 * @param y The y coordinate of the foot step in meters.
	 * @param theta The orientation of the foot step in radians.
	 * @param foot The swing foot (LEFT or RIGHT).
	 * @return The foot step node.
	 *
	 * The foot step is allocated lazily on request.
	 */
	static FootstepNode* get(const double& x, const double&y, const double& theta, const Foot& foot);

	/**
	 * Compares this foot step to another footstep.
	 * @param other The other foot steps.
	 * @return True iff the two foot steps are approximately equal.
	 */
	bool operator==(const FootstepNode& other) const;

   	/**
   	 * @brief Returns a string representation of the node for the program output.
   	 * @return The string representation.
   	 */
	std::string toString() const;

   	/**
   	 * @brief Returns a string representation of the node for logging.
   	 * @return The string representation.
   	 */
	std::string toLogString() const;

private:
	typedef int IndexType;
	typedef std::map<IndexType, FootstepNode*> MapType;
	static MapType nodes;
	FootstepNode(const double& x, const double& y, const double& theta, const Foot& foot);

};

int hash_value(const FootstepNode &node);

}

#endif /* FOOTSTEP_PLANNING_FOOTSTEPNODE_H_ */
