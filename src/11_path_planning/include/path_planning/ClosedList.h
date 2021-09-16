#ifndef PATH_PLANNING_CLOSEDLIST_HPP_
#define PATH_PLANNING_CLOSEDLIST_HPP_

#include <set>
#include <path_planning/AbstractNode.h>
#include <path_planning/FileIO.h>
#include <iostream>

namespace path_planning {

/**
 * @brief Implements a "closed list" for A* based on a hash table.
 */
class ClosedList {
public:
	ClosedList();
	virtual ~ClosedList();

	/**
	 * @brief Adds a node to the closed list.
	 * @param node The node to add.
	 * @throw std::invalid_argument node is NULL
	 *
	 * If a node is added multiple times, the method will print a warning message on stderr.
	 */
	void add(const AbstractNode * const node);

	/**
	 * @brief Checks if a node is already contained in the list.
	 * @param node The node to look for.
	 * @return True iff the node is already contained in the list.
	 * @throw std::invalid_argument node is NULL
	 */
	bool contains(const AbstractNode * const node) const;

private:
	std::set<const AbstractNode *> list;
	bool duplicateWarning;

public:
	static FileIO *fileIO;  ///< Helper for logging data.
	static bool logToStdout; ///< Write log messages to stdout.
};

}  // namespace path_planning

#endif /* PATH_PLANNING_CLOSEDLIST_HPP_ */
