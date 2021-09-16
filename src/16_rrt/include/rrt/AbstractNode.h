#ifndef RRT_ABSTRACTNODE_H_
#define RRT_ABSTRACTNODE_H_

#include <stdexcept>
#include <string>

namespace rrt {

/**
 * @brief Abstract node of an RRT.
 */
class AbstractNode {
public:
	double costs;  ///< The costs for reaching this node.
	 /**
	  * @brief Set the link to the predecessor of the current node.
	  * @param predecessor Pointer to the predecessor node.
	  */
	void setPredecessor(AbstractNode * predecessor) {
		if (predecessor == this) {
			throw std::runtime_error("Setting node as its own predecessor creates an infinite loop");
		}
		this->predecessor = predecessor;
	}
	/**
	 * @brief Returns the pointer to the predecessor node if it was previously set with setPredecessor(Abstract Node*).
	 * @return Pointer to the predecessor if set, otherwise NULL.
	 */
	AbstractNode * getPredecessor() const {
		return this->predecessor;
	}

	/**
	 * @brief Sets the link to the connection node for connecting two trees.
	 * @param connection Pointer to the connection node in the other tree.
	 */
	void setConnection(AbstractNode * connection) {
		if (connection == this) {
			throw std::runtime_error("Setting node as its own connector creates an infinite loop");
		}
		this->connection = connection;
	}

	/**
	 * @brief Returns the pointer to the connection node in the other tree if it was previously set with setConnection(AbstractNode*).
	 * @return Pointer to the connection node if set, otherwise NULL.
	 */
	AbstractNode * getConnection() const {
		return this->connection;
	}

	/**
	 * @brief Returns a human-readable representation of the node.
	 * @return String representation of the node.
	 */
	virtual std::string toString() const = 0;

	/**
	 * @brief Returns a string representation of the node for logging.
	 * @return String representation of the node.
	 */
	virtual std::string toLogString() const = 0;

	/**
	 * @brief Tests if a pointer to the predecessor was set before with setPredecessor(AbstractNode*).
	 * @return True iff the predecessor is set.
	 */
	bool predecessorExists() {
		return predecessor != NULL;
	}

	/**
	 * @brief Tests if a pointer to the connection node was set before with setConnection(AbstractNode*).
	 * @return True iff the connection node is set.
	 */
	bool connectionExists() {
		return predecessor != NULL;
	}

protected:
	AbstractNode * predecessor;  ///< Pointer to the predecessor node.
	AbstractNode * connection;   ///< Pointer to the connection node in the other tree.
	AbstractNode() :
			costs(0.0), predecessor(NULL), connection(NULL) {
	}

	virtual ~AbstractNode() {
	}

};

}  // namespace rrt

#endif /* RRT_ABSTRACTNODE_H_ */
