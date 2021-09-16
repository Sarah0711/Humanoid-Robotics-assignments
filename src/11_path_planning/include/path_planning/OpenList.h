#ifndef PATH_PLANNING_OPENLIST_HPP_
#define PATH_PLANNING_OPENLIST_HPP_

#include <map>
#include <set>
#include <queue>
#include <path_planning/FileIO.h>
#include <path_planning/AbstractNode.h>

namespace path_planning {

/**
 * @brief Implements an "open list" for A* based on a binomial heap priority queue.
 */
class OpenList {
public:
	OpenList();
	virtual ~OpenList();

	/**
	 * @brief Adds a node to the priority queue.
	 * @param node The node to add.
	 * @param costs The costs of the node.
	 * @throws std::invalid_argument if node is NULL.
	 */
	void enqueue(const AbstractNode * const node, const double costs);
	/**
	 * @brief Returns and removes the node with the minimum costs from the priority queue.
	 * @return The minimum node.
	 * @throws std::runtime_error if the queue is empty.
	 */
	const AbstractNode * removeMin();
	/**
	 * @brief Updates the costs of a node.
	 * @param node The node which should be updated.
	 * @param costs The new costs.
	 * @throws std::invalid_argument if node is NULL.
	 * @throws std::runtime_error if node is not contained in the priority queue.
	 */
	void updateCosts(const AbstractNode * const node, const double costs);
	/**
	 * @brief Tests if the priority queue is empty.
	 * @return True iff the queue is empty.
	 */
	bool isEmpty() const;
	/**
	 * @brief Tests if the priority queue contains a given node.
	 * @param node The node to search.
	 * @return True iff the queue contains the node.
	 * @throws std::invalid_argument if node is NULL.
	 */
	bool contains(const AbstractNode * const node) const;
	/**
	 * @brief Returns the costs of a given node.
	 * @param node The node.
	 * @return The costs of the given node.
	 * @throws std::invalid_argument if node is NULL.
	 * @throws std::runtime_error if node is not contained in the priority queue.
	 */
	double getCosts(const AbstractNode * const node) const;

protected:
	struct NodeWrapper;
	/**
	 * @brief Compare nodes by costs.
	 */
	struct CompareNode {
		/**
		 * @brief Compares the costs of two nodes.
		 * @param node1 The first node.
		 * @param node2 The second node.
		 * @return True iff the node1 has higher costs than node2.
		 */
		bool operator()(const NodeWrapper* const node1, const NodeWrapper* const node2) const {
			return node1->costs > node2->costs;
		}
	} compareNode;

	/**
	 * Wraps a pointer to an AbstractNode for use in a heap.
	 */
	struct NodeWrapper {
		const AbstractNode * node;  ///< Pointer to the AbstractNode.
		double costs; ///< Costs of the node.

		/**
		 * @brief Constructor
		 * @param node Pointer to the AbstractNode.
		 * @param costs Costs of the node.
		 */
		NodeWrapper(const AbstractNode * const node, const double costs) : node(node), costs(costs) {}
	};

	/**
	 * @brief Priority queue, partially ordered (heap)
	 */
	typedef std::priority_queue<NodeWrapper*, std::vector<NodeWrapper*>, CompareNode> Queue;

	/**
	 * @brief Hash map mapping for looking up the wrapped node corresponding to an AbstractNode pointer.
	 */
	typedef std::map<const AbstractNode *, NodeWrapper*> HashMap;

	mutable Queue openList;  ///< Priority queue for the open list.
	bool duplicateWarning;   ///< True if a node has been added twice to the open list.
	bool reinsertWarning;    ///< True if a node has been inserted to the open list that has already been closed.

public:
	static FileIO *fileIO;  ///< Helper for logging data.
	static bool logToStdout;      ///< Write log messages to stdout.
	static const GridMap *map;

private:
	struct OpenListData {
	    enum { NEW, OPEN, CLOSED } state;
	    double cost;
	    OpenListData() : state(NEW), cost(0.0) {}
	};
	mutable std::vector<OpenListData> data;
	OpenListData& getData(AbstractNode * node);
	OpenListData& getData(const AbstractNode * const node) const;
};

}  // namespace planning

#endif /* PATH_PLANNING_OPENLIST_HPP_ */
