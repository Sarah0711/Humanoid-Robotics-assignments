#ifndef PATH_PLANNING_ABSTRACTNODE_H_
#define PATH_PLANNING_ABSTRACTNODE_H_

#include <stdexcept>
#include <string>

namespace ara_star {

/**
 * @brief Abstract superclass for A* nodes.
 */
class AbstractNode {
public:
	double costs;  ///< The costs for reaching this node (usually called "g").

	/**
	 * @brief Sets the predecessor of the node that can later be used to reconstruct the path.
	 * @param predecessor Pointer to the predecessor of the node.
	 * @throws std::invalid_argument The node passed as the predecessor is invalid (e.g., it would create an infinite loop).
	 */
   	void setPredecessor(const AbstractNode *predecessor) {
   		if (predecessor == this) {
   			throw std::invalid_argument("Setting node as its own predecessor creates an infinite loop");
   		}
   		this->predecessor = predecessor;
   	}
   	/**
   	 * @brief Returns the predecessor of the node that was previously set using setPredecessor().
   	 * @return
   	 */
   	const AbstractNode* getPredecessor() const {
   		return this->predecessor;
   	}

   	/**
   	 * @brief Returns a string representation of the node for the program output.
   	 * @return The string representation.
   	 */
   	virtual std::string toString() const = 0;
   	/**
   	 * @brief Returns a string representation of the node for logging.
   	 * @return The string representation.
   	 */
   	virtual std::string toLogString() const = 0;

protected:
	AbstractNode() : costs(0.0), predecessor(NULL) {};
	virtual ~AbstractNode() {};

private:
	const AbstractNode *predecessor;
};

}  // namespace ara_star

#endif /* PATH_PLANNING_ABSTRACTNODE_H_ */
