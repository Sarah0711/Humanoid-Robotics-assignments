#ifndef PATH_PLANNING_FILEIO_H_
#define PATH_PLANNING_FILEIO_H_

#include <fstream>
#include <deque>
#include <footstep_planning/FootstepMap.h>
#include <footstep_planning/AbstractNode.h>
#include <footstep_planning/GridMap.h>

namespace footstep_planning {

/**
 * @brief Helper class for loading a map and logging A* events.
 */
class FileIO {
public:
	/**
	 * Loads a foot step map.
	 * @param package_path The package path.
	 */
	FileIO(const std::string& package_path);
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

	static const GridMap * loadMap(const std::string& filename);

	const FootstepMap *map; ///< The footstep map.
};

} /* namespace footstep_planning */

#endif /* PATH_PLANNING_FILEIO_H_ */
