#ifndef PATH_PLANNING_FILEIO_H_
#define PATH_PLANNING_FILEIO_H_

#include <fstream>
#include <deque>
#include <path_planning/AbstractNode.h>
#include <path_planning/GridNode.h>
#include <path_planning/GridMap.h>

namespace path_planning {

/**
 * @brief Helper class for loading a map and logging A* events.
 */
class FileIO {
public:
	FileIO();
	virtual ~FileIO();
	/**
	 * @brief Opens the log file.
	 * @param filename The filename for the log file.
	 * @return True iff the log file could be opened successfully.
	 */
	bool openLogfile(const std::string& filename);

	/**
	 * @brief Logs the final path from the robot's position to the goal to a log file.
	 * @param filename The filename for the log file.
	 * @param path The path of nodes from the robot's position to the goal as a double-ended queue.
	 */
	void logPath(const std::string& filename, std::deque<const AbstractNode *> path);
	/**
	 * @brief Logs an "open node" event when a node is added to the priority queue.
	 * @param node The node that is currently being opened.
	 */
	void logOpenNode(const AbstractNode * const node);
	/**
	 * @brief Logs a "close node" event when a node is removed from the priority queue.
	 * @param node The node that is currently being closed.
	 */
	void logCloseNode(const AbstractNode * const node);
	/**
	 * @brief Loads a grid map from a file.
	 * @param filename The filename of the map file
	 * @return Pointer to the newly allocated grid map.
	 *
	 * The caller has to call "delete" in the end to free the map.
	 */
	static const GridMap * loadMap(const std::string& filename);

private:
	std::ofstream logfile;
};

} /* namespace path_planning */

#endif /* PATH_PLANNING_FILEIO_H_ */
