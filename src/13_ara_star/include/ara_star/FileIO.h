#ifndef PATH_PLANNING_FILEIO_H_
#define PATH_PLANNING_FILEIO_H_

#include <fstream>
#include <deque>
#include <ara_star/AbstractNode.h>
#include <ara_star/GridNode.h>
#include <ara_star/GridMap.h>

namespace ara_star {

/**
 * @brief Helper class for loading a map and logging A* events.
 */
class FileIO {
public:
	FileIO();
	virtual ~FileIO();
	static const GridMap * loadMap(const std::string& filename);

};

} /* namespace ara_star */

#endif /* PATH_PLANNING_FILEIO_H_ */
