#ifndef PATH_PLANNING_ABSTRACTNODE_H_
#define PATH_PLANNING_ABSTRACTNODE_H_

#include <stdexcept>
#include <string>

namespace footstep_planning {

struct OpenListData {
private:
    enum { NEW, OPEN, CLOSED } state;
    double cost;
    OpenListData() : state(NEW), cost(0.0) {}
    friend class OpenList;
    friend class AbstractNode;
};

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
	AbstractNode() : costs(0.0), predecessor(NULL), openListData(new OpenListData()) {};
	virtual ~AbstractNode() {
		delete openListData;
		openListData = NULL;
	};

private:
	const AbstractNode *predecessor;
	OpenListData *openListData;
	friend class OpenList;
};

}  // namespace footstep_planning

#endif /* PATH_PLANNING_ABSTRACTNODE_H_ */
